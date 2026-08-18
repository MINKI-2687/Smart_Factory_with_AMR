#include "xparameters.h"
#include "xil_io.h"
#include "xstatus.h"
#include "xuartps.h"
#include "xuartps_hw.h"
#include "sleep.h"

#include "platform/platform.h"
#include "ov5640/OV5640.h"
#include "ov5640/ScuGicInterruptController.h"
#include "ov5640/PS_GPIO.h"
#include "ov5640/AXI_VDMA.h"
#include "ov5640/PS_IIC.h"

#include "MIPI_D_PHY_RX.h"
#include "MIPI_CSI_2_RX.h"


#define IRPT_CTL_DEVID 		XPAR_PS7_SCUGIC_0_DEVICE_ID
#define GPIO_DEVID			XPAR_PS7_GPIO_0_DEVICE_ID
#define GPIO_IRPT_ID			XPAR_PS7_GPIO_0_INTR
#define CAM_I2C_DEVID		XPAR_PS7_I2C_0_DEVICE_ID
#define CAM_I2C_IRPT_ID		XPAR_PS7_I2C_0_INTR
#define VDMA_DEVID			XPAR_AXIVDMA_0_DEVICE_ID
#define VDMA_MM2S_IRPT_ID	XPAR_FABRIC_AXI_VDMA_0_MM2S_INTROUT_INTR
#define VDMA_S2MM_IRPT_ID	XPAR_FABRIC_AXI_VDMA_0_S2MM_INTROUT_INTR
#define CAM_I2C_SCLK_RATE	100000

#define DDR_BASE_ADDR		XPAR_DDR_MEM_BASEADDR
#define MEM_BASE_ADDR		(DDR_BASE_ADDR + 0x0A000000)

#define GAMMA_BASE_ADDR     XPAR_AXI_GAMMACORRECTION_0_BASEADDR
#define BASIC_COORD_BASE_ADDR XPAR_BASIC_COORD_GPIO_BASEADDR
#define BASIC_COORD_SELECT_OFFSET 0x08U
#define BASIC_COORD_TRI2_OFFSET 0x0CU
#define BASIC_MAX_OBJECTS 6U
#define BASIC_SR04_BASE_ADDR XPAR_BASIC_SR04_GPIO_BASEADDR
#define BASIC_SR04_SIGNATURE 0x53U
#define BASIC_SR04_CAR_PRESENT_MASK (1U << 17)
#define BASIC_RUN_ENABLE_MASK (1U << 19)

#define BASIC_AUTOMATIC_ROBOT_FLOW 1
#define BASIC_ROBOT_UART_BAUD 115200U
#define BASIC_ACK_WAIT_MS 100U

#define BASIC_PACKET_SOF0 0xAAU
#define BASIC_PACKET_SOF1 0x55U
#define BASIC_PACKET_TARGET 0x01U
#define BASIC_PACKET_ACK 0x02U
#define BASIC_PACKET_RETRY 0x03U
#define BASIC_TARGET_PACKET_SIZE 9U

using namespace digilent;

struct BasicTarget {
	u16 x;
	u16 y;
	u8 index;
	bool valid;
};

struct BasicAckParser {
	u8 state;
	u8 type;
	u8 sequence;
};

static u8 basic_crc8_update(u8 crc, u8 data)
{
	crc ^= data;
	for (u32 bit = 0U; bit < 8U; ++bit)
		crc = (crc & 0x80U) ? (u8)((crc << 1) ^ 0x07U) : (u8)(crc << 1);
	return crc;
}

static u8 basic_crc8(const u8 *data, u32 length)
{
	u8 crc = 0U;
	for (u32 index = 0U; index < length; ++index)
		crc = basic_crc8_update(crc, data[index]);
	return crc;
}

static bool basic_car_present()
{
	const u32 status = Xil_In32(BASIC_SR04_BASE_ADDR);
	return ((status >> 24) == BASIC_SR04_SIGNATURE) &&
			((status & BASIC_SR04_CAR_PRESENT_MASK) != 0U);
}

static bool basic_run_enabled()
{
	const u32 status = Xil_In32(BASIC_SR04_BASE_ADDR);
	return ((status >> 24) == BASIC_SR04_SIGNATURE) &&
			((status & BASIC_RUN_ENABLE_MASK) != 0U);
}

static bool basic_read_target(u32 index, BasicTarget& target)
{
	Xil_Out32(BASIC_COORD_BASE_ADDR + BASIC_COORD_SELECT_OFFSET, index);
	usleep(2U);

	const u32 result = Xil_In32(BASIC_COORD_BASE_ADDR);
	const u32 selected = (result >> 26) & 0x7U;
	target.x = (u16)(result & 0x7ffU);
	target.y = (u16)((result >> 11) & 0x7ffU);
	target.index = (u8)index;
	target.valid = (((result >> 22) & 0x1U) != 0U) && (selected == index);
	return target.valid;
}

static bool basic_select_next_target(BasicTarget& selected_target)
{
	BasicTarget candidate;
	selected_target.valid = false;

	for (u32 index = 0U; index < BASIC_MAX_OBJECTS; ++index) {
		if (!basic_read_target(index, candidate))
			continue;

		if (!selected_target.valid ||
				(candidate.y < selected_target.y) ||
				((candidate.y == selected_target.y) &&
				 (candidate.x < selected_target.x))) {
			selected_target = candidate;
		}
	}

	return selected_target.valid;
}

static bool basic_robot_uart_init(XUartPs& uart)
{
	XUartPs_Config *config = XUartPs_LookupConfig(XPAR_PS7_UART_0_DEVICE_ID);
	if (config == 0)
		return false;

	if (XUartPs_CfgInitialize(&uart, config, config->BaseAddress) != XST_SUCCESS)
		return false;
	if (XUartPs_SetBaudRate(&uart, BASIC_ROBOT_UART_BAUD) != XST_SUCCESS)
		return false;

	XUartPs_SetOperMode(&uart, XUARTPS_OPER_MODE_NORMAL);
	XUartPs_SetOptions(&uart, XUARTPS_OPTION_RESET_TX | XUARTPS_OPTION_RESET_RX);
	return true;
}

static void basic_uart_send(XUartPs& uart, const u8 *data, u32 length)
{
	u32 sent = 0U;
	while (sent < length)
		sent += XUartPs_Send(&uart, const_cast<u8 *>(&data[sent]), length - sent);
	while (XUartPs_IsSending(&uart) != 0U) {
	}
}

static void basic_send_target_packet(
		XUartPs& uart, u8 sequence, const BasicTarget& target)
{
	u8 packet[BASIC_TARGET_PACKET_SIZE];
	packet[0] = BASIC_PACKET_SOF0;
	packet[1] = BASIC_PACKET_SOF1;
	packet[2] = BASIC_PACKET_TARGET;
	packet[3] = sequence;
	packet[4] = (u8)(target.x & 0xffU);
	packet[5] = (u8)(target.x >> 8);
	packet[6] = (u8)(target.y & 0xffU);
	packet[7] = (u8)(target.y >> 8);
	packet[8] = basic_crc8(&packet[2], 6U);
	basic_uart_send(uart, packet, BASIC_TARGET_PACKET_SIZE);
}

static u8 basic_response_parser_push(
		BasicAckParser& parser, u8 byte, u8 expected_sequence)
{
	switch (parser.state) {
	case 0U:
		parser.state = (byte == BASIC_PACKET_SOF0) ? 1U : 0U;
		break;
	case 1U:
		if (byte == BASIC_PACKET_SOF1)
			parser.state = 2U;
		else
			parser.state = (byte == BASIC_PACKET_SOF0) ? 1U : 0U;
		break;
	case 2U:
		parser.type = byte;
		parser.state = 3U;
		break;
	case 3U:
		parser.sequence = byte;
		parser.state = 4U;
		break;
	default: {
		u8 payload[2] = {parser.type, parser.sequence};
		const bool valid_crc = byte == basic_crc8(payload, 2U);
		const bool valid_type =
				(parser.type == BASIC_PACKET_ACK) ||
				(parser.type == BASIC_PACKET_RETRY);
		const bool valid = valid_crc && valid_type &&
				(parser.sequence == expected_sequence);
		const u8 response = valid ? parser.type : 0U;
		parser.state = 0U;
		return response;
	}
	}
	return 0U;
}

static u8 basic_poll_for_response(
		XUartPs& uart, BasicAckParser& parser, u8 expected_sequence)
{
	const u32 base = uart.Config.BaseAddress;
	while (XUartPs_IsReceiveData(base)) {
		const u8 byte = (u8)XUartPs_ReadReg(base, XUARTPS_FIFO_OFFSET);
		const u8 response = basic_response_parser_push(
				parser, byte, expected_sequence);
		if (response != 0U)
			return response;
	}
	return 0U;
}

static void basic_run_robot_flow()
{
	XUartPs robot_uart;
	BasicAckParser ack_parser = {0U, 0U, 0U};
	BasicTarget target = {0U, 0U, 0U, false};
	u8 sequence = 0U;

	if (!basic_robot_uart_init(robot_uart)) {
		xil_printf("ERROR: Robot UART0 initialization failed.\r\n");
		while (1) {
		}
	}

	Xil_Out32(BASIC_COORD_BASE_ADDR + BASIC_COORD_TRI2_OFFSET, 0U);
	Xil_Out32(BASIC_COORD_BASE_ADDR + BASIC_COORD_SELECT_OFFSET, 0U);

	xil_printf("Robot flow ready: set SW0=1 to run; LED0 shows run enable.\r\n");

	while (1) {
		while (!basic_run_enabled())
			usleep(20000U);

		while (basic_run_enabled() && !basic_car_present())
			usleep(20000U);

		if (!basic_run_enabled()) {
			xil_printf("SW0 OFF: robot flow paused.\r\n");
			continue;
		}

		while (basic_run_enabled() && basic_car_present() &&
				!basic_select_next_target(target))
			usleep(50000U);
		if (!basic_run_enabled()) {
			xil_printf("SW0 OFF: robot flow paused.\r\n");
			continue;
		}
		if (!basic_car_present())
			continue;

		xil_printf("TARGET seq=%d index=%d X=%d Y=%d\r\n",
				(int)sequence, (int)target.index,
				(int)target.x, (int)target.y);

		u32 attempt = 0U;
		bool acknowledged = false;
		while (!acknowledged) {
			++attempt;
			basic_send_target_packet(robot_uart, sequence, target);

			for (u32 wait_ms = 0U; wait_ms < BASIC_ACK_WAIT_MS; ++wait_ms) {
				const u8 response = basic_poll_for_response(
						robot_uart, ack_parser, sequence);
				if (response == BASIC_PACKET_ACK) {
					acknowledged = true;
					break;
				}
				usleep(1000U);
			}

			if (!acknowledged) {
				xil_printf("ACK timeout: seq=%d attempt=%d\r\n",
						(int)sequence, (int)attempt);
				if ((attempt % 3U) == 0U)
					usleep(900000U);
			}
		}

		xil_printf("ACK received: seq=%d, target TX locked.\r\n",
				(int)sequence);

		bool retry_requested = false;
		while (basic_car_present()) {
			const u8 response = basic_poll_for_response(
					robot_uart, ack_parser, sequence);
			if (response == BASIC_PACKET_RETRY) {
				retry_requested = true;
				break;
			}
			usleep(20000U);
		}

		if (retry_requested) {
			++sequence;
			xil_printf(
					"RETRY received: selecting a fresh target with seq=%d.\r\n",
					(int)sequence);
			usleep(200000U);
			continue;
		}

		xil_printf("RC car departed.\r\n");

		while (!basic_car_present())
			usleep(20000U);
		xil_printf("RC car returned. Selecting current targets.\r\n");

		++sequence;
		usleep(200000U);

		if (!basic_run_enabled())
			xil_printf("SW0 OFF: current cycle complete, robot flow paused.\r\n");
	}
}

static void print_basic_coordinate()
{
	u32 result;
	u32 count;
	u32 printed = 0U;

	Xil_Out32(BASIC_COORD_BASE_ADDR + BASIC_COORD_TRI2_OFFSET, 0U);
	Xil_Out32(BASIC_COORD_BASE_ADDR + BASIC_COORD_SELECT_OFFSET, 0U);
	result = Xil_In32(BASIC_COORD_BASE_ADDR);
	count = (result >> 23) & 0x7U;
	xil_printf("Blue targets confirmed: %d/6\r\n", (int)count);

	for (u32 index = 0U; index < BASIC_MAX_OBJECTS; ++index) {
		Xil_Out32(BASIC_COORD_BASE_ADDR + BASIC_COORD_SELECT_OFFSET, index);
		for (u32 attempt = 0U; attempt < 4U; ++attempt) {
			result = Xil_In32(BASIC_COORD_BASE_ADDR);
			if (((result >> 26) & 0x7U) == index)
				break;
		}

		const u32 x = result & 0x7FFU;
		const u32 y = (result >> 11) & 0x7FFU;
		const u32 valid = (result >> 22) & 0x1U;
		const u32 selected = (result >> 26) & 0x7U;
		const u32 stable = (result >> 29) & 0x7U;

		if (valid && (selected == index)) {
			xil_printf("  #%d: X=%d, Y=%d, stable=%d/3\r\n",
					(int)(index + 1U), (int)x, (int)y, (int)stable);
			++printed;
		}
	}

	if (printed == 0U)
		xil_printf("  No stable blue target detected.\r\n");
}

void pipeline_mode_change(AXI_VDMA<ScuGicInterruptController>& vdma_driver, OV5640& cam, VideoOutput& vid, Resolution res, OV5640_cfg::mode_t mode)
{
	//Bring up input pipeline back-to-front
	{
		vdma_driver.resetWrite();
		MIPI_CSI_2_RX_mWriteReg(XPAR_MIPI_CSI_2_RX_0_S_AXI_LITE_BASEADDR, CR_OFFSET, (CR_RESET_MASK & ~CR_ENABLE_MASK));
		MIPI_D_PHY_RX_mWriteReg(XPAR_MIPI_D_PHY_RX_0_S_AXI_LITE_BASEADDR, CR_OFFSET, (CR_RESET_MASK & ~CR_ENABLE_MASK));
		cam.reset();
	}

	{
		vdma_driver.configureWrite(timing[static_cast<int>(res)].h_active, timing[static_cast<int>(res)].v_active);
		Xil_Out32(GAMMA_BASE_ADDR, 3); // Set Gamma correction factor to 1/1.8
		//TODO CSI-2, D-PHY config here
		cam.init();
	}

	{
		vdma_driver.enableWrite();
		MIPI_CSI_2_RX_mWriteReg(XPAR_MIPI_CSI_2_RX_0_S_AXI_LITE_BASEADDR, CR_OFFSET, CR_ENABLE_MASK);
		MIPI_D_PHY_RX_mWriteReg(XPAR_MIPI_D_PHY_RX_0_S_AXI_LITE_BASEADDR, CR_OFFSET, CR_ENABLE_MASK);
		cam.set_mode(mode);
		cam.set_awb(OV5640_cfg::awb_t::AWB_ADVANCED);
	}

	//Bring up output pipeline back-to-front
	{
		vid.reset();
		vdma_driver.resetRead();
	}

	{
		vid.configure(res);
		vdma_driver.configureRead(timing[static_cast<int>(res)].h_active, timing[static_cast<int>(res)].v_active);
	}

	{
		vid.enable();
		vdma_driver.enableRead();
	}
}

int main()
{
	init_platform();

	ScuGicInterruptController irpt_ctl(IRPT_CTL_DEVID);
	PS_GPIO<ScuGicInterruptController> gpio_driver(GPIO_DEVID, irpt_ctl, GPIO_IRPT_ID);
	PS_IIC<ScuGicInterruptController> iic_driver(CAM_I2C_DEVID, irpt_ctl, CAM_I2C_IRPT_ID, 100000);

	OV5640 cam(iic_driver, gpio_driver);
	AXI_VDMA<ScuGicInterruptController> vdma_driver(VDMA_DEVID, MEM_BASE_ADDR, irpt_ctl,
			VDMA_MM2S_IRPT_ID,
			VDMA_S2MM_IRPT_ID);
	VideoOutput vid(XPAR_VTC_0_DEVICE_ID, XPAR_VIDEO_DYNCLK_DEVICE_ID);

	pipeline_mode_change(vdma_driver, cam, vid, Resolution::R1920_1080_60_PP, OV5640_cfg::mode_t::MODE_1080P_1920_1080_30fps);


	xil_printf("Video init done.\r\n");

#if BASIC_AUTOMATIC_ROBOT_FLOW
	basic_run_robot_flow();
#endif


	// Liquid lens control
	uint8_t read_char0 = 0;
	uint8_t read_char1 = 0;
	uint8_t read_char2 = 0;
	uint8_t read_char4 = 0;
	uint8_t read_char5 = 0;
	uint16_t reg_addr;
	uint8_t reg_value;

	while (1) {
		xil_printf("\r\n\r\n\r\nPcam 5C MAIN OPTIONS\r\n");
		xil_printf("\r\nPlease press the key corresponding to the desired option:");
		xil_printf("\r\n  a. Change Resolution");
		xil_printf("\r\n  b. Change Liquid Lens Focus");
		xil_printf("\r\n  d. Change Image Format (Raw or RGB)");
		xil_printf("\r\n  e. Write a Register Inside the Image Sensor");
		xil_printf("\r\n  f. Read a Register Inside the Image Sensor");
		xil_printf("\r\n  g. Change Gamma Correction Factor Value");
		xil_printf("\r\n  h. Change AWB Settings");
		xil_printf("\r\n  i. Print up to 6 blue target coordinates\r\n\r\n");

		read_char0 = getchar();
		getchar();
		xil_printf("Read: %d\r\n", read_char0);

		switch(read_char0) {

		case 'a':
			xil_printf("\r\n  Please press the key corresponding to the desired resolution:");
			xil_printf("\r\n    1. 1280 x 720, 60fps");
			xil_printf("\r\n    2. 1920 x 1080, 15fps");
			xil_printf("\r\n    3. 1920 x 1080, 30fps");
			read_char1 = getchar();
			getchar();
			xil_printf("\r\nRead: %d", read_char1);
			switch(read_char1) {
			case '1':
				pipeline_mode_change(vdma_driver, cam, vid, Resolution::R1280_720_60_PP, OV5640_cfg::mode_t::MODE_720P_1280_720_60fps);
				xil_printf("Resolution change done.\r\n");
				break;
			case '2':
				pipeline_mode_change(vdma_driver, cam, vid, Resolution::R1920_1080_60_PP, OV5640_cfg::mode_t::MODE_1080P_1920_1080_15fps);
				xil_printf("Resolution change done.\r\n");
				break;
			case '3':
				pipeline_mode_change(vdma_driver, cam, vid, Resolution::R1920_1080_60_PP, OV5640_cfg::mode_t::MODE_1080P_1920_1080_30fps);
				xil_printf("Resolution change done.\r\n");
				break;
			default:
				xil_printf("\r\n  Selection is outside the available options! Please retry...");
			}
			break;

		case 'b':
			xil_printf("\r\n\r\nPlease enter value of liquid lens register, in hex, with small letters: 0x");
			//A, B, C,..., F need to be entered with small letters
			while (read_char1 < 48) {
				read_char1 = getchar();
			}
			while (read_char2 < 48) {
				read_char2 = getchar();
			}
			getchar();
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char1 <= 57) {
				read_char1 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char1 -= 87;
			}
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char2 <= 57) {
				read_char2 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char2 -= 87;
			}
			cam.writeRegLiquid((uint8_t) (16*read_char1 + read_char2));
			xil_printf("\r\nWrote to liquid lens controller: %x", (uint8_t) (16*read_char1 + read_char2));
			break;

		case 'd':
			xil_printf("\r\n  Please press the key corresponding to the desired setting:");
			xil_printf("\r\n    1. Select image format to be RGB, output still Raw");
			xil_printf("\r\n    2. Select image format & output to both be Raw");
			read_char1 = getchar();
			getchar();
			xil_printf("\r\nRead: %d", read_char1);
			switch(read_char1) {
			case '1':
				cam.set_isp_format(OV5640_cfg::isp_format_t::ISP_RGB);
				xil_printf("Settings change done.\r\n");
				break;
			case '2':
				cam.set_isp_format(OV5640_cfg::isp_format_t::ISP_RAW);
				xil_printf("Settings change done.\r\n");
				break;
			default:
				xil_printf("\r\n  Selection is outside the available options! Please retry...");
			}
			break;

		case 'e':
			xil_printf("\r\nPlease enter address of image sensor register, in hex, with small letters: \r\n");
			//A, B, C,..., F need to be entered with small letters
			while (read_char1 < 48) {
				read_char1 = getchar();
			}
			while (read_char2 < 48) {
				read_char2 = getchar();
			}
			while (read_char4 < 48) {
				read_char4 = getchar();
			}
			while (read_char5 < 48) {
				read_char5 = getchar();
			}
			getchar();
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char1 <= 57) {
				read_char1 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char1 -= 87;
			}
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char2 <= 57) {
				read_char2 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char2 -= 87;
			}
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char4 <= 57) {
				read_char4 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char4 -= 87;
			}
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char5 <= 57) {
				read_char5 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char5 -= 87;
			}
			reg_addr = 16*(16*(16*read_char1 + read_char2)+read_char4)+read_char5;
			xil_printf("Desired Register Address: %x\r\n", reg_addr);

			read_char1 = 0;
			read_char2 = 0;
			xil_printf("\r\nPlease enter value of image sensor register, in hex, with small letters: \r\n");
			//A, B, C,..., F need to be entered with small letters
			while (read_char1 < 48) {
				read_char1 = getchar();
			}
			while (read_char2 < 48) {
				read_char2 = getchar();
			}
			getchar();
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char1 <= 57) {
				read_char1 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char1 -= 87;
			}
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char2 <= 57) {
				read_char2 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char2 -= 87;
			}
			reg_value = 16*read_char1 + read_char2;
			xil_printf("Desired Register Value: %x\r\n", reg_value);
			cam.writeReg(reg_addr, reg_value);
			xil_printf("Register write done.\r\n");

			break;

		case 'f':
			xil_printf("Please enter address of image sensor register, in hex, with small letters: \r\n");
			//A, B, C,..., F need to be entered with small letters
			while (read_char1 < 48) {
				read_char1 = getchar();
			}
			while (read_char2 < 48) {
				read_char2 = getchar();
			}
			while (read_char4 < 48) {
				read_char4 = getchar();
			}
			while (read_char5 < 48) {
				read_char5 = getchar();
			}
			getchar();
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char1 <= 57) {
				read_char1 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char1 -= 87;
			}
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char2 <= 57) {
				read_char2 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char2 -= 87;
			}
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char4 <= 57) {
				read_char4 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char4 -= 87;
			}
			// If character is a digit, convert from ASCII code to a digit between 0 and 9
			if (read_char5 <= 57) {
				read_char5 -= 48;
			}
			// If character is a letter, convert ASCII code to a number between 10 and 15
			else {
				read_char5 -= 87;
			}
			reg_addr = 16*(16*(16*read_char1 + read_char2)+read_char4)+read_char5;
			xil_printf("Desired Register Address: %x\r\n", reg_addr);

			cam.readReg(reg_addr, reg_value);
			xil_printf("Value of Desired Register: %x\r\n", reg_value);

			break;

		case 'g':
			xil_printf("  Please press the key corresponding to the desired Gamma factor:\r\n");
			xil_printf("    1. Gamma Factor = 1\r\n");
			xil_printf("    2. Gamma Factor = 1/1.2\r\n");
			xil_printf("    3. Gamma Factor = 1/1.5\r\n");
			xil_printf("    4. Gamma Factor = 1/1.8\r\n");
			xil_printf("    5. Gamma Factor = 1/2.2\r\n");
			read_char1 = getchar();
			getchar();
			xil_printf("Read: %d\r\n", read_char1);
			// Convert from ASCII to numeric
			read_char1 = read_char1 - 48;
			if ((read_char1 > 0) && (read_char1 < 6)) {
				Xil_Out32(GAMMA_BASE_ADDR, read_char1-1);
				xil_printf("Gamma value changed to 1.\r\n");
			}
			else {
				xil_printf("  Selection is outside the available options! Please retry...\r\n");
			}
			break;

		case 'h':
			xil_printf("  Please press the key corresponding to the desired AWB change:\r\n");
			xil_printf("    1. Enable Advanced AWB\r\n");
			xil_printf("    2. Enable Simple AWB\r\n");
			xil_printf("    3. Disable AWB\r\n");
			read_char1 = getchar();
			getchar();
			xil_printf("Read: %d\r\n", read_char1);
			switch(read_char1) {
			case '1':
				cam.set_awb(OV5640_cfg::awb_t::AWB_ADVANCED);
				xil_printf("Enabled Advanced AWB\r\n");
				break;
			case '2':
				cam.set_awb(OV5640_cfg::awb_t::AWB_SIMPLE);
				xil_printf("Enabled Simple AWB\r\n");
				break;
			case '3':
				cam.set_awb(OV5640_cfg::awb_t::AWB_DISABLED);
				xil_printf("Disabled AWB\r\n");
				break;
			default:
				xil_printf("  Selection is outside the available options! Please retry...\r\n");
			}
			break;

		case 'i':
			print_basic_coordinate();
			break;

		default:
			xil_printf("  Selection is outside the available options! Please retry...\r\n");
		}

		read_char1 = 0;
		read_char2 = 0;
		read_char4 = 0;
		read_char5 = 0;
	}


	cleanup_platform();

	return 0;
}
