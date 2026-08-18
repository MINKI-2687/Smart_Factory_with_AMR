#ifndef SMOOTHING_H
#define SMOOTHING_H

#include <math.h>
#include <stdlib.h>
#include "path.h"  /* Path, path_push, path_init 사용 - A star 알고리즘 자체와는 무관해서 이것만 있으면 됨 */
void path_smooth(const Path *path, double weight_data, double weight_smooth,
                  double tolerance, int max_iterations, Path *out);

#endif /* SMOOTHING_H */
