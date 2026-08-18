#include "min_heap.h"


void heap_init(MinHeap *h) {
    h->nodes = NULL; h->count = 0; h->capacity = 0;
}


void heap_free(MinHeap *h) {
    free(h->nodes); h->nodes = NULL; h->count = 0; h->capacity = 0;
}


void heap_swap(HeapNode *a, HeapNode *b) {
    HeapNode tmp = *a; *a = *b; *b = tmp;
}


void heap_push(MinHeap *h, double f_score, int row, int col) {
    if (h->count >= h->capacity) {
        h->capacity = (h->capacity == 0) ? 64 : h->capacity * 2;
        h->nodes = (HeapNode *)realloc(h->nodes, sizeof(HeapNode) * h->capacity);
    }
    int i = h->count++;
    h->nodes[i] = (HeapNode){ f_score, row, col };
    while (i > 0) {
        int parent = (i - 1) / 2;
        if (h->nodes[parent].f_score <= h->nodes[i].f_score) break;
        heap_swap(&h->nodes[parent], &h->nodes[i]);
        i = parent;
    }
}


bool heap_empty(const MinHeap *h) {
    return h->count == 0;
}


HeapNode heap_pop(MinHeap *h) {
    HeapNode top = h->nodes[0];
    h->count--;
    h->nodes[0] = h->nodes[h->count];
    int i = 0;
    while (true) {
        int left = 2 * i + 1, right = 2 * i + 2, smallest = i;
        if (left < h->count && h->nodes[left].f_score < h->nodes[smallest].f_score) smallest = left;
        if (right < h->count && h->nodes[right].f_score < h->nodes[smallest].f_score) smallest = right;
        if (smallest == i) break;
        heap_swap(&h->nodes[i], &h->nodes[smallest]);
        i = smallest;
    }
    return top;
}
