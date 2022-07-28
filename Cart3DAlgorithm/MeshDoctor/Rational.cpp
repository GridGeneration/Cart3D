#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#pragma warning(disable:4267)
#pragma warning(disable:4244)
#pragma warning(disable:4018)
#endif
#include<MeshDoctor/Rational.h>


namespace Cart3DAlgorithm
{

    namespace
    {
#define FFMIN(a,b) ((a) > (b) ? (b) : (a))
#define FFSWAP(type,a,b) do{type SWAP_tmp= b; b= a; a= SWAP_tmp;}while(0)
#ifndef ff_ctzll
#define ff_ctzll ff_ctzll_c
        /* We use the De-Bruijn method outlined in:
         * http://supertech.csail.mit.edu/papers/debruijn.pdf. */
        static inline int ff_ctzll_c(long long v)
        {
            static const uint8_t debruijn_ctz64[64] = {
                0, 1, 2, 53, 3, 7, 54, 27, 4, 38, 41, 8, 34, 55, 48, 28,
                62, 5, 39, 46, 44, 42, 22, 9, 24, 35, 59, 56, 49, 18, 29, 11,
                63, 52, 6, 26, 37, 40, 33, 47, 61, 45, 43, 21, 23, 58, 17, 10,
                51, 25, 36, 32, 60, 20, 57, 16, 50, 31, 19, 15, 30, 14, 13, 12
            };
            return debruijn_ctz64[(uint64_t)((v & -v) * 0x022FDD63CC95386DU) >> 58];
        }
#endif
        int64_t av_gcd(int64_t a, int64_t b) {
            int za, zb, k;
            int64_t u, v;
            if (a == 0)
                return b;
            if (b == 0)
                return a;
            za = ff_ctzll(a);
            zb = ff_ctzll(b);
            k = FFMIN(za, zb);
            u = llabs(a >> za);
            v = llabs(b >> zb);
            while (u != v) {
                if (u > v)
                    FFSWAP(int64_t, v, u);
                v -= u;
                v >>= ff_ctzll(v);
            }
            return (uint64_t)u << k;
        }

        int av_reduce(crational::int_type* dst_num, crational::int_type* dst_den,
            int64_t num, int64_t den, int64_t max)
        {
            std::pair<crational::int_type, crational::int_type> a0 = { 0, 1 }, a1 = { 1, 0 };
            int sign = (num < 0) ^ (den < 0);
            int64_t gcd = av_gcd(std::abs(num), std::abs(den));

            if (gcd) {
                num = std::abs(num) / gcd;
                den = std::abs(den) / gcd;
            }
            if (num <= max && den <= max) {
                a1 = std::pair<crational::int_type, crational::int_type>{ num, den };
                den = 0;
            }

            while (den) {
                uint64_t x = num / den;
                int64_t next_den = num - den * x;
                int64_t a2n = x * a1.first + a0.first;
                int64_t a2d = x * a1.second + a0.second;

                if (a2n > max || a2d > max) {
                    if (a1.first) x = (max - a0.first) / a1.first;
                    if (a1.second) x = FFMIN(x, (max - a0.second) / a1.second);

                    if (den * (2 * x * a1.second + a0.second) > num * a1.second)
                        a1 = std::pair<crational::int_type, crational::int_type>{
                        x * a1.first + a0.first,
                        x * a1.second + a0.second };
                    break;
                }
                a0 = a1;
                a1 = std::pair<crational::int_type, crational::int_type>{ a2n, a2d };
                num = den;
                den = next_den;
            }
            *dst_num = sign ? -a1.first : a1.first;
            *dst_den = a1.second;
            return den == 0;
        }

    }


	crational Rational::fromcfloat(cfloat d, int max)
	{
        crational a;
        int exponent;
        int64_t den;
        if (isnan(d))
            return crational{ 0, 0 };
        if (fabs(d) > INT_MAX + 3LL)
            return crational{ d < 0 ? -1 : 1, 0 };
        double res=frexp(d, &exponent);
        exponent = std::max(exponent - 1, 0);
        den = 1LL << (61 - exponent);
        crational::int_type anum, aden;
        anum = a.numerator();
        aden = a.denominator();
        av_reduce(&anum, &aden, floor(d * den + 0.5), den, max);
        if ((!anum || !aden) && d && max > 0 && max < INT_MAX)
            av_reduce(&anum, &aden, floor(d * den + 0.5), den, INT_MAX);
        a = crational(anum, aden);
        return a;
	}



}



#if defined(_MSC_VER)
#pragma warning(pop)
#endif