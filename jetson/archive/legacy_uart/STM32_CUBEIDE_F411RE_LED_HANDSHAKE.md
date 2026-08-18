# NUCLEO-F411RE UART LED 핸드셰이크

이 단계에서는 다이나믹셀 쉴드와 로봇팔을 연결하지 않는다. Nucleo의 LD2 LED로
Jetson 명령 수신, ACK, DONE만 확인한다.

## 물리 연결

```text
개발 PC USB -------- Nucleo CN1(ST-LINK USB): 펌웨어 다운로드 + 보드 전원
Jetson USB ---------- 두 번째 PL2303 USB-TTL
                          TXD(초록) -> Nucleo D2/PA10/USART1_RX
                          RXD(흰색) <- Nucleo D8/PA9/USART1_TX
                          GND(검정) <-> Nucleo GND
                          +5V(빨강) 연결 금지
```

USART1을 쓰므로 다이나믹셀 쉴드 납땜이나 Nucleo의 solder bridge 변경은 필요 없다.

## STM32CubeIDE 1.18 설정

1. `File > New > STM32 Project > Board Selector`에서 `NUCLEO-F411RE` 선택.
2. 기본 보드 초기화를 허용한다.
3. `PA5`를 `GPIO_Output`으로 설정한다. 이것이 내장 LD2다.
4. `Connectivity > USART1 > Asynchronous`를 선택한다.
5. 핀이 `PA9=USART1_TX`, `PA10=USART1_RX`인지 확인한다.
6. `115200`, `8 data bits`, `No parity`, `1 stop bit`, flow control 없음으로 설정한다.
7. `NVIC Settings > USART1 global interrupt`를 활성화한다.
8. 코드를 생성한 후 아래 내용을 `main.c`의 해당 USER CODE 영역에 추가한다.

### `/* USER CODE BEGIN PV */`

```c
#define UART_SOF_1       0xAA
#define UART_SOF_2       0x55
#define UART_STATUS_ACK  0x06
#define UART_STATUS_DONE 0x07
#define UART_STATUS_NAK  0x15

static uint8_t uart_rx_byte;
static uint8_t uart_rx_frame[5];
static volatile uint8_t uart_rx_index = 0;
static volatile uint8_t uart_command_pending = 0;
static volatile uint8_t uart_pending_id = 0;
static volatile uint8_t uart_pending_slot = 0;

static uint8_t uart_have_last = 0;
static uint8_t uart_last_id = 0;
static uint8_t uart_last_slot = 0;
```

### `/* USER CODE BEGIN 0 */`

```c
static uint8_t crc8_atm(const uint8_t *data, uint32_t length)
{
  uint8_t crc = 0;
  for (uint32_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint32_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U)
                          : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

static void send_status(uint8_t command_id, uint8_t status)
{
  uint8_t response[5] = {UART_SOF_1, UART_SOF_2, command_id, status, 0};
  response[4] = crc8_atm(&response[2], 2);
  HAL_UART_Transmit(&huart1, response, sizeof(response), 100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1) {
    return;
  }

  uint8_t value = uart_rx_byte;
  if (uart_rx_index == 0 && value != UART_SOF_1) {
    /* 헤더가 아니면 버린다. */
  } else if (uart_rx_index == 1 && value != UART_SOF_2) {
    uart_rx_index = (value == UART_SOF_1) ? 1 : 0;
    uart_rx_frame[0] = value;
  } else {
    uart_rx_frame[uart_rx_index++] = value;
    if (uart_rx_index == sizeof(uart_rx_frame)) {
      uint8_t command_id = uart_rx_frame[2];
      uint8_t slot = uart_rx_frame[3];
      uint8_t valid =
          (slot >= 1 && slot <= 9) &&
          (uart_rx_frame[4] == crc8_atm(&uart_rx_frame[2], 2));

      if (valid && !uart_command_pending) {
        uart_pending_id = command_id;
        uart_pending_slot = slot;
        uart_command_pending = 1;
      } else if (!valid) {
        send_status(command_id, UART_STATUS_NAK);
      }
      uart_rx_index = 0;
    }
  }

  HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
}
```

### `MX_USART1_UART_Init();` 다음 `/* USER CODE BEGIN 2 */`

```c
HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
```

### `while (1)` 안의 `/* USER CODE BEGIN 3 */`

```c
if (uart_command_pending) {
  uint8_t command_id;
  uint8_t slot;

  __disable_irq();
  command_id = uart_pending_id;
  slot = uart_pending_slot;
  uart_command_pending = 0;
  __enable_irq();

  if (uart_have_last && command_id == uart_last_id) {
    if (slot == uart_last_slot) {
      /* Jetson 재시도: 응답만 반복하고 LED 동작은 반복하지 않는다. */
      send_status(command_id, UART_STATUS_ACK);
      send_status(command_id, UART_STATUS_DONE);
    } else {
      send_status(command_id, UART_STATUS_NAK);
    }
  } else {
    send_status(command_id, UART_STATUS_ACK);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_Delay(150);

    uart_last_id = command_id;
    uart_last_slot = slot;
    uart_have_last = 1;
    send_status(command_id, UART_STATUS_DONE);
  }
}
```

위 예제는 연결 시험용이다. 실제 로봇팔 단계에서는 LED 토글 자리를 모터 동작
요청으로 바꾸고, 동작이 완전히 끝난 뒤에만 DONE을 보내야 한다.

## Jetson 실행

먼저 FPGA와 카메라 없이 STM32 링크만 시험한다.

```bash
cd /home/aidl/work/rack_dataset_v1
python3 tools/communication/stm32_handshake_test.py /dev/ttyUSB1 --slot 3
```

LD2가 한 번 토글되고 `ACK`, `DONE`, `PASS`가 순서대로 나오면 UART 링크가
정상이다. 그다음 통합 실행으로 넘어간다.

카메라 모니터를 첫 터미널에서 실행하고, 통합 컨트롤러를 두 번째 터미널에서
실행한다. 단독 `zybo_slot_receiver.py`는 동시에 실행하지 않는다.

```bash
ls -l /dev/ttyUSB*

cd /home/aidl/work/rack_dataset_v1
python3 tools/communication/robot_dispatch_controller.py \
  --fpga-port /dev/ttyUSB0 \
  --stm32-port /dev/ttyUSB1
```

성공 로그는 `STM32 TX`, `status=ACK`, `status=DONE`, `DISPATCH DONE` 순서다.
