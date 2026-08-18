#include "map_io.h"


/* ============================================================
 * map_io: SLAM으로 한 번 만든(또는 고정맵으로 구성한) 점유격자를 파일로 저장/불러오기.
 * "매핑과 내비게이션은 완전히 독립적"이라는 원래 의도를 살리기 위함 - 맵은 한 번만
 * 만들고, 그 뒤로는 프로그램을 몇 번을 새로 켜도 저장된 맵을 그대로 불러와서 바로
 * 내비게이션(셔틀 왕복)만 하면 됨.
 *
 * 파일 형식 (텍스트, 사람이 읽어도 알아볼 수 있게):
 *   줄1: rows cols resolution
 *   줄2부터: 각 행이 그대로 0/1 문자열 (예: "0001100...")
 * ============================================================ */

bool map_save(const char *path, const unsigned char *grid,
                             int rows, int cols, double resolution) {
    FILE *f = fopen(path, "w");
    if (!f) {
        fprintf(stderr, "[map_io] 저장 실패: %s 를 열 수 없음\n", path);
        return false;
    }
    fprintf(f, "%d %d %.6f\n", rows, cols, resolution);
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            fputc(grid[r * cols + c] ? '1' : '0', f);
        }
        fputc('\n', f);
    }
    fclose(f);
    printf("[map_io] 맵 저장 완료: %s (%d x %d, resolution=%.3f)\n", path, rows, cols, resolution);
    return true;
}


/* 실패하면 NULL 반환. 성공하면 호출부가 free()해야 함. */
unsigned char *map_load(const char *path, int *out_rows, int *out_cols,
                                       double *out_resolution) {
    FILE *f = fopen(path, "r");
    if (!f) {
        fprintf(stderr, "[map_io] 불러오기 실패: %s 를 열 수 없음\n", path);
        return NULL;
    }
    int rows, cols;
    double resolution;
    if (fscanf(f, "%d %d %lf\n", &rows, &cols, &resolution) != 3) {
        fprintf(stderr, "[map_io] 불러오기 실패: %s 형식이 이상함 (첫 줄 파싱 실패)\n", path);
        fclose(f);
        return NULL;
    }
    unsigned char *grid = (unsigned char *)malloc((size_t)rows * cols);
    for (int r = 0; r < rows; r++) {
        char line[4096];
        if (!fgets(line, sizeof(line), f)) {
            fprintf(stderr, "[map_io] 불러오기 실패: %s 의 %d번째 행을 못 읽음\n", path, r);
            free(grid);
            fclose(f);
            return NULL;
        }
        for (int c = 0; c < cols; c++) {
            grid[r * cols + c] = (line[c] == '1') ? 1 : 0;
        }
    }
    fclose(f);
    *out_rows = rows; *out_cols = cols; *out_resolution = resolution;
    printf("[map_io] 맵 불러오기 완료: %s (%d x %d, resolution=%.3f)\n", path, rows, cols, resolution);
    return grid;
}
