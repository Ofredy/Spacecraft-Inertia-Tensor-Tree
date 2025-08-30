#include "quat_math.h"

static inline double sqr(double x) { return x * x; }

std::array<double,4> quat_normalize(const std::array<double,4>& q) 
{
    const double n2 = sqr(q[0]) + sqr(q[1]) + sqr(q[2]) + sqr(q[3]);
    if (n2 <= 0.0) throw std::runtime_error("quat_normalize: zero-norm quaternion");
    const double invn = 1.0 / std::sqrt(n2);
    return { q[0]*invn, q[1]*invn, q[2]*invn, q[3]*invn };
}

std::array<double,4> quat_conjugate(const std::array<double,4>& q) 
{
    return { q[0], -q[1], -q[2], -q[3] };
}

std::array<double,4> quat_inverse(const std::array<double,4>& q) 
{
    const double n2 = sqr(q[0]) + sqr(q[1]) + sqr(q[2]) + sqr(q[3]);
    if (n2 <= 0.0) throw std::runtime_error("quat_inverse: zero-norm quaternion");
    const double invn2 = 1.0 / n2;
    auto qc = quat_conjugate(q);
    return { qc[0]*invn2, qc[1]*invn2, qc[2]*invn2, qc[3]*invn2 };
}

// Hamilton product q = q2 ⊗ q1
std::array<double,4> quat_multiply(const std::array<double,4>& q2,
                                   const std::array<double,4>& q1) 
{
    const double w2=q2[0], x2=q2[1], y2=q2[2], z2=q2[3];
    const double w1=q1[0], x1=q1[1], y1=q1[2], z1=q1[3];

    return {
        w2*w1 - x2*x1 - y2*y1 - z2*z1, // w
        w2*x1 + x2*w1 + y2*z1 - z2*y1, // x
        w2*y1 - x2*z1 + y2*w1 + z2*x1, // y
        w2*z1 + x2*y1 - y2*x1 + z2*w1  // z
    };
}

std::array<double,4> quat_between(const std::array<double,4>& q_from,
                                  const std::array<double,4>& q_to) 
{
    auto q_rel = quat_multiply(quat_normalize(q_to), quat_inverse(quat_normalize(q_from)));
    return quat_normalize(q_rel);
}

mat3 quat_to_dcm(const std::array<double,4>& q_in) 
{
    auto q = quat_normalize(q_in);
    const double w=q[0], x=q[1], y=q[2], z=q[3];

    const double ww=w*w, xx=x*x, yy=y*y, zz=z*z;
    const double wx=w*x, wy=w*y, wz=w*z;
    const double xy=x*y, xz=x*z, yz=y*z;

    mat3 R{{
        { ww + xx - yy - zz,   2*(xy - wz),       2*(xz + wy) },
        { 2*(xy + wz),         ww - xx + yy - zz, 2*(yz - wx) },
        { 2*(xz - wy),         2*(yz + wx),       ww - xx - yy + zz }
    }};
    return R;
}

static inline mat3 matmul(const mat3& A, const mat3& B) 
{
    mat3 C{};
    for (int i=0;i<3;++i) 
    {
        for (int j=0;j<3;++j) 
        {
            double s = 0.0;
            for (int k=0;k<3;++k) s += A[i][k]*B[k][j];
            C[i][j] = s;
        }
    }
    return C;
}

static inline mat3 transpose(const mat3& A) 
{
    mat3 AT{};
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) AT[i][j] = A[j][i];
    return AT;
}

mat3 rotate_inertia(const mat3& I_old, const std::array<double,4>& q_rot) 
{
    const mat3 R  = quat_to_dcm(q_rot);
    const mat3 RT = transpose(R);
    const mat3 tmp = matmul(R, I_old);
    mat3 I_new = matmul(tmp, RT);

    // enforce symmetry
    for (int i=0;i<3;++i) 
    {
        for (int j=i+1;j<3;++j) 
        {
            const double sym = 0.5*(I_new[i][j] + I_new[j][i]);
            I_new[i][j] = I_new[j][i] = sym;
        }
    }
    return I_new;
}
