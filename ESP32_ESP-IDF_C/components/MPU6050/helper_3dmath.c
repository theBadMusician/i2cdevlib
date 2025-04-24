/* helper_3dmath.c */
#include "helper_3dmath.h"

/* ==== Quaternion ==== */

Quaternion quaternion_product(const Quaternion* a, const Quaternion* b) {
    return quaternion_create(
        a->w*b->w - a->x*b->x - a->y*b->y - a->z*b->z,
        a->w*b->x + a->x*b->w + a->y*b->z - a->z*b->y,
        a->w*b->y - a->x*b->z + a->y*b->w + a->z*b->x,
        a->w*b->z + a->x*b->y - a->y*b->x + a->z*b->w
    );
}

Quaternion quaternion_conjugate(const Quaternion* q) {
    return quaternion_create(q->w, -q->x, -q->y, -q->z);
}

float quaternion_magnitude(const Quaternion* q) {
    return sqrtf(q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z);
}

void quaternion_normalize(Quaternion* q) {
    float m = quaternion_magnitude(q);
    if (m != 0.0f) {
        q->w /= m; q->x /= m; q->y /= m; q->z /= m;
    }
}

Quaternion quaternion_normalized(const Quaternion* q) {
    Quaternion r = *q;
    quaternion_normalize(&r);
    return r;
}


/* ==== VectorInt16 ==== */

float vector_int16_magnitude(const VectorInt16* v) {
    return sqrtf((float)v->x*v->x + (float)v->y*v->y + (float)v->z*v->z);
}

void vector_int16_normalize(VectorInt16* v) {
    float m = vector_int16_magnitude(v);
    if (m != 0.0f) {
        v->x = (int16_t)(v->x / m);
        v->y = (int16_t)(v->y / m);
        v->z = (int16_t)(v->z / m);
    }
}

VectorInt16 vector_int16_normalized(const VectorInt16* v) {
    VectorInt16 r = *v;
    vector_int16_normalize(&r);
    return r;
}

void vector_int16_rotate(VectorInt16* v, const Quaternion* q) {
    /* P_out = q * P_in * conj(q) */
    Quaternion p = quaternion_create(0.0f, v->x, v->y, v->z);
    Quaternion t = quaternion_product(q, &p);
    Quaternion c = quaternion_conjugate(q);
    Quaternion r = quaternion_product(&t, &c);
    v->x = (int16_t)r.x;
    v->y = (int16_t)r.y;
    v->z = (int16_t)r.z;
}

VectorInt16 vector_int16_rotated(const VectorInt16* v, const Quaternion* q) {
    VectorInt16 r = *v;
    vector_int16_rotate(&r, q);
    return r;
}


/* ==== VectorFloat ==== */

float vector_float_magnitude(const VectorFloat* v) {
    return sqrtf(v->x*v->x + v->y*v->y + v->z*v->z);
}

void vector_float_normalize(VectorFloat* v) {
    float m = vector_float_magnitude(v);
    if (m != 0.0f) {
        v->x /= m; v->y /= m; v->z /= m;
    }
}

VectorFloat vector_float_normalized(const VectorFloat* v) {
    VectorFloat r = *v;
    vector_float_normalize(&r);
    return r;
}

void vector_float_rotate(VectorFloat* v, const Quaternion* q) {
    Quaternion p = quaternion_create(0.0f, v->x, v->y, v->z);
    Quaternion t = quaternion_product(q, &p);
    Quaternion c = quaternion_conjugate(q);
    Quaternion r = quaternion_product(&t, &c);
    v->x = r.x;
    v->y = r.y;
    v->z = r.z;
}

VectorFloat vector_float_rotated(const VectorFloat* v, const Quaternion* q) {
    VectorFloat r = *v;
    vector_float_rotate(&r, q);
    return r;
}

