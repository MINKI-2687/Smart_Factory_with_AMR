#ifndef MIN_HEAP_H
#define MIN_HEAP_H

#include <stdlib.h>
#include <stdbool.h>

typedef struct {
    double f_score;
    int row, col;
} HeapNode;

typedef struct {
    HeapNode *nodes;
    int count;
    int capacity;
} MinHeap;
void heap_init(MinHeap *h);
void heap_free(MinHeap *h);
void heap_swap(HeapNode *a, HeapNode *b);
void heap_push(MinHeap *h, double f_score, int row, int col);
bool heap_empty(const MinHeap *h);
HeapNode heap_pop(MinHeap *h);

#endif /* MIN_HEAP_H */
