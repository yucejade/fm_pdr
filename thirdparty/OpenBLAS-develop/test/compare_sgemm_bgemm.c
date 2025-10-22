/***************************************************************************
Copyright (c) 2025 The OpenBLAS Project
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in
the documentation and/or other materials provided with the
distribution.
3. Neither the name of the OpenBLAS project nor the names of
its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE OPENBLAS PROJECT OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************/
#include "../common.h"
#include <stdint.h>
#include <stdio.h>

#include "test_helpers.h"

#define SGEMM BLASFUNC(sgemm)
#define BGEMM BLASFUNC(bgemm)
#define BGEMM_LARGEST 256

int
main (int argc, char *argv[])
{
  blasint m, n, k;
  int i, j, l;
  blasint x, y;
  blasint one = 1;
  int ret = 0;
  int loop = BGEMM_LARGEST;
  char transA = 'N', transB = 'N';
  float alpha = 1.0, beta = 1.0;
  bfloat16 alpha_bf16;
  sbstobf16_(&one, &alpha, &one, &alpha_bf16, &one);
  bfloat16 beta_bf16;
  sbstobf16_(&one, &beta, &one, &beta_bf16, &one);

  for (x = 0; x <= loop; x++)
  {
    if ((x > 100) && (x != BGEMM_LARGEST)) continue;
    m = k = n = x;
    float *A = (float *)malloc_safe(m * k * sizeof(FLOAT));
    float *B = (float *)malloc_safe(k * n * sizeof(FLOAT));
    float *C = (float *)malloc_safe(m * n * sizeof(FLOAT));
    bfloat16 *AA = (bfloat16 *)malloc_safe(m * k * sizeof(bfloat16));
    bfloat16 *BB = (bfloat16 *)malloc_safe(k * n * sizeof(bfloat16));
    bfloat16 *CC = (bfloat16 *)malloc_safe(k * n * sizeof(bfloat16));
    FLOAT *DD = (FLOAT *)malloc_safe(m * n * sizeof(FLOAT));
    if ((A == NULL) || (B == NULL) || (C == NULL) || (AA == NULL) || (BB == NULL) ||
        (DD == NULL) || (CC == NULL))
      return 1;

    for (j = 0; j < m; j++)
    {
      for (i = 0; i < k; i++)
      {
        A[j * k + i] = ((FLOAT) rand () / (FLOAT) RAND_MAX) + 0.5;
        sbstobf16_(&one, &A[j*k+i], &one, &AA[j * k + i], &one);
      }
    }
    for (j = 0; j < n; j++)
    {
      for (i = 0; i < k; i++)
      {
        B[j * k + i] = ((FLOAT) rand () / (FLOAT) RAND_MAX) + 0.5;
        sbstobf16_(&one, &B[j*k+i], &one, &BB[j * k + i], &one);
      }
    }
    for (y = 0; y < 4; y++)
    {
      if ((y == 0) || (y == 2)) {
        transA = 'N';
      } else {
        transA = 'T';
      }
      if ((y == 0) || (y == 1)) {
        transB = 'N';
      } else {
        transB = 'T';
      }

      for (j = 0; j < m; j++)
      {
        for (i = 0; i < n; i++)
        {
          C[j * n + i] = 100.0;
          DD[j * n + i] = 100.0;
          sbstobf16_(&one, &C[j * n + i], &one, &CC[j * n + i], &one);
        }
      }

      SGEMM (&transA, &transB, &m, &n, &k, &alpha, A,
        &m, B, &k, &beta, C, &m);
      BGEMM (&transA, &transB, &m, &n, &k, &alpha_bf16, (bfloat16*)AA,
        &m, (bfloat16*)BB, &k, &beta_bf16, (bfloat16*)CC, &m);

      for (i = 0; i < n; i++)
        for (j = 0; j < m; j++)
        {
          for (l = 0; l < k; l++)
            if (transA == 'N' && transB == 'N')
            {
              DD[i * m + j] +=
                float16to32 (AA[l * m + j]) * float16to32 (BB[l + k * i]);
            } else if (transA == 'T' && transB == 'N')
            {
              DD[i * m + j] +=
                float16to32 (AA[k * j + l]) * float16to32 (BB[l + k * i]);
            } else if (transA == 'N' && transB == 'T')
            {
              DD[i * m + j] +=
                float16to32 (AA[l * m + j]) * float16to32 (BB[i + l * n]);
            } else if (transA == 'T' && transB == 'T')
            {
              DD[i * m + j] +=
                float16to32 (AA[k * j + l]) * float16to32 (BB[i + l * n]);
            }
          if (!is_close(float16to32(CC[i * m + j]), truncate_float32_to_bfloat16(C[i * m + j]), 0.01, 0.001)) {
#ifdef DEBUG
            printf("Mismatch at i=%d, j=%d, k=%d: CC=%.6f, C=%.6f\n",
                    i, j, k, float16to32(CC[i * m + j]), truncate_float32_to_bfloat16(C[i * m + j]));
#endif
            ret++;
          }

          if (!is_close(float16to32(CC[i * m + j]), truncate_float32_to_bfloat16(DD[i * m + j]), 0.01, 0.001)) {
#ifdef DEBUG
            printf("Mismatch at i=%d, j=%d, k=%d: CC=%.6f, DD=%.6f\n",
                    i, j, k, float16to32(CC[i * m + j]), truncate_float32_to_bfloat16(DD[i * m + j]));
#endif
            ret++;
          }
            
        }
    }
    free(A);
    free(B);
    free(C);
    free(AA);
    free(BB);
    free(CC);
    free(DD);
  }

  if (ret != 0) {
    fprintf(stderr, "BGEMM FAILURES: %d\n", ret);
    return 1;
  }

  return ret;
}
