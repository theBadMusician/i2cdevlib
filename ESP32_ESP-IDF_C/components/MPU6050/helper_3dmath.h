/* helper_3dmath.h */
#ifndef HELPER_3DMATH_H
#define HELPER_3DMATH_H

#include <math.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float w, x, y, z;
} Quaternion;

/* Constructors */
static inline Quaternion quaternion_create(float w, float x, float y, float z) {
    Quaternion q = { w, x, y, z };
    return q;
}

/* Operations */
Quaternion    quaternion_product    (const Quaternion* a, const Quaternion* b);
Quaternion    quaternion_conjugate  (const Quaternion* q);
float         quaternion_magnitude  (const Quaternion* q);
void          quaternion_normalize  (Quaternion* q);
Quaternion    quaternion_normalized (const Quaternion* q);


typedef struct {
    int16_t x, y, z;
} VectorInt16;

/* Constructors */
static inline VectorInt16 vector_int16_create(int16_t x, int16_t y, int16_t z) {
    VectorInt16 v = { x, y, z };
    return v;
}

/* Operations */
float         vector_int16_magnitude  (const VectorInt16* v);
void          vector_int16_normalize  (VectorInt16* v);
VectorInt16   vector_int16_normalized (const VectorInt16* v);
void          vector_int16_rotate     (VectorInt16* v, const Quaternion* q);
VectorInt16   vector_int16_rotated    (const VectorInt16* v, const Quaternion* q);


typedef struct {
    float x, y, z;
} VectorFloat;

/* Constructors */
static inline VectorFloat vector_float_create(float x, float y, float z) {
    VectorFloat v = { x, y, z };
    return v;
}

/* Operations */
float         vector_float_magnitude  (const VectorFloat* v);
void          vector_float_normalize  (VectorFloat* v);
VectorFloat   vector_float_normalized (const VectorFloat* v);
void          vector_float_rotate     (VectorFloat* v, const Quaternion* q);
VectorFloat   vector_float_rotated    (const VectorFloat* v, const Quaternion* q);

#ifdef __cplusplus
}
#endif

#endif /* HELPER_3DMATH_H */

