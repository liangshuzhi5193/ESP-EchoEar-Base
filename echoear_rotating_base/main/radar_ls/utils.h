// Copyright 2022 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

bool mac_str2hex(const char *mac_str, uint8_t *mac_hex);
float avg(const float *array, size_t len);
float sum(const float *a, size_t len);
float max(const float *array, size_t len, float percent);
float min(const float *array, size_t len, float percent);
float trimmean(const float *array, size_t len, float percent);


float cov(const float *x, const float *y, size_t len);
float corr(const float *a, const float *b, size_t len);
float std(const float *a, size_t len);
float dis(const float *a, const float *b, size_t len);
int cmp_float(const void *_a, const void *_b);
float median(const float *a, size_t len);

esp_err_t pca(uint32_t cols,
              uint32_t row_0, const float data_0[row_0][cols],
              uint32_t row_1, const float data_1[row_1][cols],
              float output[cols]);

#ifdef __cplusplus
}
#endif
