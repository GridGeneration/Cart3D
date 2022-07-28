#pragma once

// This file comes with a companion unit test, `rationals_test.C`. Use it!

// What follows are the leading comments from the Boost project's rational.hpp,
// from which this file is copied and altered.
// This file is based off of commit 47c1b26964b06a99200769474d58a309267c50f1,
// accessed on May 3, 2020 using the URL:
//
//     https://github.com/boostorg/rational/blob/
//         47c1b26964b06a99200769474d58a309267c50f1/include/boost/rational.hpp

//  (C) Copyright Paul Moore 1999. Permission to copy, use, modify, sell and
//  distribute this software is granted provided this copyright notice appears
//  in all copies. This software is provided "as is" without express or
//  implied warranty, and with no claim as to its suitability for any purpose.

// boostinspect:nolicense (don't complain about the lack of a Boost license)
// (Paul Moore hasn't been in contact for years, so there's no way to change the
// license.)

//  See http://www.boost.org/libs/rational for documentation.

//  Credits:
//  Thanks to the boost mailing list in general for useful comments.
//  Particular contributions included:
//    Andrew D Jewell, for reminding me to take care to avoid overflow
//    Ed Brey, for many comments, including picking up on some dreadful typos
//    Stephen Silver contributed the test suite and comments on user-defined
//    IntType
//    Nickolay Mladenov, for the implementation of operator+=

//  Revision History
//  02 Sep 13  Remove unneeded forward declarations; tweak private helper
//             function (Daryle Walker)
//  30 Aug 13  Improve exception safety of "assign"; start modernizing I/O code
//             (Daryle Walker)
//  27 Aug 13  Add cross-version constructor template, plus some private helper
//             functions; add constructor to exception class to take custom
//             messages (Daryle Walker)
//  25 Aug 13  Add constexpr qualification wherever possible (Daryle Walker)
//  05 May 12  Reduced use of implicit gcd (Mario Lang)
//  05 Nov 06  Change rational_cast to not depend on division between different
//             types (Daryle Walker)
//  04 Nov 06  Off-load GCD and LCM to Boost.Integer; add some invariant checks;
//             add std::numeric_limits<> requirement to help GCD (Daryle Walker)
//  31 Oct 06  Recoded both operator< to use round-to-negative-infinity
//             divisions; the rational-value version now uses continued fraction
//             expansion to avoid overflows, for bug #798357 (Daryle Walker)
//  20 Oct 06  Fix operator bool_type for CW 8.3 (Joaquín M López Muñoz)
//  18 Oct 06  Use EXPLICIT_TEMPLATE_TYPE helper macros from Boost.Config
//             (Joaquín M López Muñoz)
//  27 Dec 05  Add Boolean conversion operator (Daryle Walker)
//  28 Sep 02  Use _left versions of operators from operators.hpp
//  05 Jul 01  Recode gcd(), avoiding std::swap (Helmut Zeisel)
//  03 Mar 01  Workarounds for Intel C++ 5.0 (David Abrahams)
//  05 Feb 01  Update operator>> to tighten up input syntax
//  05 Feb 01  Final tidy up of gcd code prior to the new release
//  27 Jan 01  Recode abs() without relying on abs(IntType)
//  21 Jan 01  Include Nickolay Mladenov's operator+= algorithm,
//             tidy up a number of areas, use newer features of operators.hpp
//             (reduces space overhead to zero), add operator!,
//             introduce explicit mixed-mode arithmetic operations
//  12 Jan 01  Include fixes to handle a user-defined IntType better
//  19 Nov 00  Throw on divide by zero in operator /= (John (EBo) David)
//  23 Jun 00  Incorporate changes from Mark Rodgers for Borland C++
//  22 Jun 00  Change _MSC_VER to BOOST_MSVC so other compilers are not
//             affected (Beman Dawes)
//   6 Mar 00  Fix operator-= normalization, #include <string> (Jens Maurer)
//  14 Dec 99  Modifications based on comments from the boost list
//  09 Dec 99  Initial Version (Paul Moore)

#include <iomanip>   // for std::setw
#include <ios>       // for std::noskipws, streamsize
#include <istream>   // for std::istream
#include <ostream>   // for std::ostream
#include <sstream>   // for std::ostringstream
#include <cstddef>   // for NULL
#include <stdexcept> // for std::domain_error
#include <string>    // for std::string implicit constructor
#include <cstdlib>   // for std::abs
#include <limits>    // for std::numeric_limits
#include <cassert>   // for assert

namespace rationals {

template <typename IntType>
IntType
gcd(IntType a, IntType b) {
    using std::abs;
    a = abs(a);
    b = abs(b);

    IntType remainder;
    while (b) {
        remainder = a % b;
        a = b;
        b = remainder;
    }

    return a;
}

class bad_rational : public std::domain_error {
  public:
    explicit bad_rational()
        : std::domain_error("bad rational: zero denominator") {}
    explicit bad_rational(char const* what) : std::domain_error(what) {}
};

template <typename IntType>
class rational {
    // Class-wide pre-conditions
    static_assert(
        ::std::numeric_limits<IntType>::is_specialized,
        "rational's type argument must have a std::numeric_limits "
        "specialization");

  public:
    // Component type
    typedef IntType int_type;

    rational() : num(0), den(1) {}

    template <class T>
    rational(const T& n) : num(n), den(1) {}

    template <class T, class U>
    rational(const T& n, const U& d) : num(n), den(d) {
        normalize();
    }

    template <typename NewType>
    explicit rational(rational<NewType> const& r)
        : num(r.numerator()),
          den(is_normalized(int_type(r.numerator()), int_type(r.denominator()))
                  ? r.denominator()
                  : (throw bad_rational(
                         "bad rational: denormalized conversion"),
                     0)) {}

    // Add assignment from IntType
    template <class T>
    rational&
    operator=(const T& n) {
        return assign(static_cast<IntType>(n), static_cast<IntType>(1));
    }

    // Assign in place
    template <class T, class U>
    rational&
    assign(const T& n, const U& d) {
        return *this = rational<IntType>(
                   static_cast<IntType>(n), static_cast<IntType>(d));
    }

    // Access to representation

    const IntType&
    numerator() const {
        return num;
    }

    const IntType&
    denominator() const {
        return den;
    }

    // Arithmetic assignment operators
    rational& operator+=(const rational& r);
    rational& operator-=(const rational& r);
    rational& operator*=(const rational& r);
    rational& operator/=(const rational& r);

    template <class T>
    rational&
    operator+=(const T& i) {
        num += i * den;
        return *this;
    }
    template <class T>
    rational&
    operator-=(const T& i) {
        num -= i * den;
        return *this;
    }
    template <class T>
    rational&
    operator*=(const T& i) {
        // Avoid overflow and preserve normalization
        IntType gcd = gcd(static_cast<IntType>(i), den);
        num *= i / gcd;
        den /= gcd;
        return *this;
    }
    template <class T>
    rational&
    operator/=(const T& i) {
        // Avoid repeated construction
        IntType const zero(0);

        if (i == zero)
            throw bad_rational();
        if (num == zero)
            return *this;

        // Avoid overflow and preserve normalization
        IntType const gcd = gcd(num, static_cast<IntType>(i));
        num /= gcd;
        den *= i / gcd;

        if (den < zero) {
            num = -num;
            den = -den;
        }

        return *this;
    }

    // Increment and decrement
    const rational&
    operator++() {
        num += den;
        return *this;
    }
    const rational&
    operator--() {
        num -= den;
        return *this;
    }

    rational
    operator++(int) {
        rational t(*this);
        ++(*this);
        return t;
    }
    rational
    operator--(int) {
        rational t(*this);
        --(*this);
        return t;
    }

    // Boolean conversion
    explicit operator bool() const {
        return num;
    }

    // Comparison operators
    bool operator<(const rational& r) const;
    bool
    operator>(const rational& r) const {
        return r < *this;
    }

    bool operator==(const rational& r) const;

    template <class T>
    bool
    operator<(const T& i) const {
        // Avoid repeated construction
        int_type const zero(0);

        // Break value into mixed-fraction form, w/ always-nonnegative remainder
        assert(this->den > zero);
        int_type q = this->num / this->den, r = this->num % this->den;
        while (r < zero) {
            r += this->den;
            --q;
        }

        // Compare with just the quotient, since the remainder always bumps the
        // value up.  [Since q = floor(n/d), and if n/d < i then q < i, if n/d == i
        // then q == i, if n/d == i + r/d then q == i, and if n/d >= i + 1 then
        // q >= i + 1 > i; therefore n/d < i iff q < i.]
        return q < i;
    }
    template <class T>
    bool
    operator>(const T& i) const {
        return operator==(i) ? false : !operator<(i);
    }
    template <class T>
    bool
    operator==(const T& i) const {
        return ((den == IntType(1)) && (num == i));
    }

  private:
    // Implementation - numerator and denominator (normalized).
    // Other possibilities - separate whole-part, or sign, fields?
    IntType num;
    IntType den;

    // Helper functions
    static int_type
    inner_gcd(IntType a, IntType b, int_type const& zero = int_type(0)) {
        return b == zero ? a : inner_gcd(b, a % b, zero);
    }

    static int_type
    inner_abs(IntType x, int_type const& zero = int_type(0)) {
        return x < zero ? -x : +x;
    }

    // Representation note: Fractions are kept in normalized form at all
    // times. normalized form is defined as gcd(num,den) == 1 and den > 0.
    // In particular, note that the implementation of abs() below relies
    // on den always being positive.
    bool test_invariant() const;
    void normalize();

    static bool
    is_normalized(
        IntType n,
        IntType d,
        int_type const& zero = int_type(0),
        int_type const& one = int_type(1)) {
        return d > zero && (n != zero || d == one) &&
               inner_abs(inner_gcd(n, d, zero), zero) == one;
    }
};

// Unary plus and minus
template <typename IntType>
inline rational<IntType>
operator+(const rational<IntType>& r) {
    return r;
}

template <typename IntType>
inline rational<IntType>
operator-(const rational<IntType>& r) {
    return rational<IntType>(
        static_cast<IntType>(-r.numerator()), r.denominator());
}

// Arithmetic assignment operators
template <typename IntType>
rational<IntType>&
rational<IntType>::operator+=(const rational<IntType>& r) {
    // This calculation avoids overflow, and minimises the number of expensive
    // calculations. Thanks to Nickolay Mladenov for this algorithm.
    //
    // Proof:
    // We have to compute a/b + c/d, where gcd(a,b)=1 and gcd(b,c)=1.
    // Let g = gcd(b,d), and b = b1*g, d=d1*g. Then gcd(b1,d1)=1
    //
    // The result is (a*d1 + c*b1) / (b1*d1*g).
    // Now we have to normalize this ratio.
    // Let's assume h | gcd((a*d1 + c*b1), (b1*d1*g)), and h > 1
    // If h | b1 then gcd(h,d1)=1 and hence h|(a*d1+c*b1) => h|a.
    // But since gcd(a,b1)=1 we have h=1.
    // Similarly h|d1 leads to h=1.
    // So we have that h | gcd((a*d1 + c*b1) , (b1*d1*g)) => h|g
    // Finally we have gcd((a*d1 + c*b1), (b1*d1*g)) = gcd((a*d1 + c*b1), g)
    // Which proves that instead of normalizing the result, it is better to
    // divide num and den by gcd((a*d1 + c*b1), g)

    // Protect against self-modification
    IntType r_num = r.num;
    IntType r_den = r.den;

    IntType g = gcd(den, r_den);
    den /= g; // = b1 from the calculations above
    num = num * (r_den / g) + r_num * den;
    g = gcd(num, g);
    num /= g;
    den *= r_den / g;

    return *this;
}

template <typename IntType>
rational<IntType>&
rational<IntType>::operator-=(const rational<IntType>& r) {
    // Protect against self-modification
    IntType r_num = r.num;
    IntType r_den = r.den;

    // This calculation avoids overflow, and minimises the number of expensive
    // calculations. It corresponds exactly to the += case above
    IntType g = gcd(den, r_den);
    den /= g;
    num = num * (r_den / g) - r_num * den;
    g = gcd(num, g);
    num /= g;
    den *= r_den / g;

    return *this;
}

template <typename IntType>
rational<IntType>&
rational<IntType>::operator*=(const rational<IntType>& r) {
    // Protect against self-modification
    IntType r_num = r.num;
    IntType r_den = r.den;

    // Avoid overflow and preserve normalization
    IntType gcd1 = gcd(num, r_den);
    IntType gcd2 = gcd(r_num, den);
    num = (num / gcd1) * (r_num / gcd2);
    den = (den / gcd2) * (r_den / gcd1);
    return *this;
}

template <typename IntType>
rational<IntType>&
rational<IntType>::operator/=(const rational<IntType>& r) {
    // Protect against self-modification
    IntType r_num = r.num;
    IntType r_den = r.den;

    // Avoid repeated construction
    IntType zero(0);

    // Trap division by zero
    if (r_num == zero)
        throw bad_rational();
    if (num == zero)
        return *this;

    // Avoid overflow and preserve normalization
    IntType gcd1 = gcd(num, r_num);
    IntType gcd2 = gcd(r_den, den);
    num = (num / gcd1) * (r_den / gcd2);
    den = (den / gcd2) * (r_num / gcd1);

    if (den < zero) {
        num = -num;
        den = -den;
    }
    return *this;
}

// Non-member operators
//
// There are three possible cases for each operator:
// 1) rational op rational
// 2) rational op other
// 3) other op rational
//
// FUTURE: Provide support for operations between rationals of different integer
//         types, the resulting type being of the wider kind.
//
// The `DEFINE_OPERATOR` macro consolidates boilerplate for the three overloads
// of each operator definition. It expands to definitions of the three overloads
// for a given operator.
// - Its first argument is the operator, e.g. `+`.
// - Its second argument is the return type, e.g. `rational<IntType>` for
//   operators that combine rationals, and `bool` for operators that compare
//   them.
// - Its third argument is the expression to `return` in the
//   `(rational, rational)` overload and the `(rational, other)` overload. `a`
//   is bound to the first argument of each function, and `b` to the second.
// - Its fourth argument is the expression to `return` in the
//   `(other, rational)` overload. This is likely a different expression, since
//   the first argument (`a`) is not a rational.

#define DEFINE_OPERATOR(OPERATOR, RETURN_TYPE, VALUE, OPPOSITE_VALUE)          \
    template <class IntType>                                                   \
    RETURN_TYPE operator OPERATOR(rational<IntType> a, rational<IntType> b) {  \
        return (VALUE);                                                        \
    }                                                                          \
                                                                               \
    template <class IntType, class Arg>                                        \
    RETURN_TYPE operator OPERATOR(rational<IntType> a, Arg b) {                \
        return (VALUE);                                                        \
    }                                                                          \
                                                                               \
    template <class IntType, class Arg>                                        \
    RETURN_TYPE operator OPERATOR(Arg a, rational<IntType> b) {                \
        return (OPPOSITE_VALUE);                                               \
    }

DEFINE_OPERATOR(+, rational<IntType>, (a += rational<IntType>(b)), (b + a))
DEFINE_OPERATOR(-, rational<IntType>, (a -= rational<IntType>(b)), -(b - a))
DEFINE_OPERATOR(*, rational<IntType>, (a *= rational<IntType>(b)), (b * a))
DEFINE_OPERATOR(
    /,
    rational<IntType>,
    (a /= rational<IntType>(b)),
    (rational<IntType>(a) /= b))
DEFINE_OPERATOR(<=, bool, (!(a > b)), (b >= a))
DEFINE_OPERATOR(>=, bool, (!(a < b)), (b <= a))
DEFINE_OPERATOR(!=, bool, (!(a == b)), (!(a == b)))

#undef DEFINE_OPERATOR

// The following free operator overloads forward to a method on their second
// (rational) argument. Note that the argument `b` comes _before_ `a`.
template <class Arg, class IntType>
bool
operator<(const Arg& b, const rational<IntType>& a) {
    return a > b;
}
template <class Arg, class IntType>
bool
operator>(const Arg& b, const rational<IntType>& a) {
    return a < b;
}
template <class Arg, class IntType>
bool
operator==(const Arg& b, const rational<IntType>& a) {
    return a == b;
}

// Comparison operators
template <typename IntType>
bool
rational<IntType>::operator<(const rational<IntType>& r) const {
    // Avoid repeated construction
    int_type const zero(0);

    // This should really be a class-wide invariant.  The reason for these
    // checks is that for 2's complement systems, INT_MIN has no corresponding
    // positive, so negating it during normalization keeps it INT_MIN, which
    // is bad for later calculations that assume a positive denominator.
    assert(this->den > zero);
    assert(r.den > zero);

    // Determine relative order by expanding each value to its simple continued
    // fraction representation using the Euclidian GCD algorithm.
    struct {
        int_type n, d, q, r;
    } ts = {this->num,
            this->den,
            static_cast<int_type>(this->num / this->den),
            static_cast<int_type>(this->num % this->den)},
      rs = {r.num,
            r.den,
            static_cast<int_type>(r.num / r.den),
            static_cast<int_type>(r.num % r.den)};
    unsigned reverse = 0u;

    // Normalize negative moduli by repeatedly adding the (positive) denominator
    // and decrementing the quotient.  Later cycles should have all positive
    // values, so this only has to be done for the first cycle.  (The rules of
    // C++ require a nonnegative quotient & remainder for a nonnegative dividend
    // & positive divisor.)
    while (ts.r < zero) {
        ts.r += ts.d;
        --ts.q;
    }
    while (rs.r < zero) {
        rs.r += rs.d;
        --rs.q;
    }

    // Loop through and compare each variable's continued-fraction components
    for (;;) {
        // The quotients of the current cycle are the continued-fraction
        // components.  Comparing two c.f. is comparing their sequences,
        // stopping at the first difference.
        if (ts.q != rs.q) {
            // Since reciprocation changes the relative order of two variables,
            // and c.f. use reciprocals, the less/greater-than test reverses
            // after each index.  (Start w/ non-reversed @ whole-number place.)
            return reverse ? ts.q > rs.q : ts.q < rs.q;
        }

        // Prepare the next cycle
        reverse ^= 1u;

        if ((ts.r == zero) || (rs.r == zero)) {
            // At least one variable's c.f. expansion has ended
            break;
        }

        ts.n = ts.d;
        ts.d = ts.r;
        ts.q = ts.n / ts.d;
        ts.r = ts.n % ts.d;
        rs.n = rs.d;
        rs.d = rs.r;
        rs.q = rs.n / rs.d;
        rs.r = rs.n % rs.d;
    }

    // Compare infinity-valued components for otherwise equal sequences
    if (ts.r == rs.r) {
        // Both remainders are zero, so the next (and subsequent) c.f.
        // components for both sequences are infinity.  Therefore, the sequences
        // and their corresponding values are equal.
        return false;
    } else {
        // Exactly one of the remainders is zero, so all following c.f.
        // components of that variable are infinity, while the other variable
        // has a finite next c.f. component.  So that other variable has the
        // lesser value (modulo the reversal flag!).
        return (ts.r != zero) != static_cast<bool>(reverse);
    }
}

template <typename IntType>
bool
rational<IntType>::operator==(const rational<IntType>& r) const {
    return ((num == r.num) && (den == r.den));
}

// Invariant check
template <typename IntType>
bool
rational<IntType>::test_invariant() const {
    return (this->den > int_type(0)) &&
           (gcd(this->num, this->den) == int_type(1));
}

// Normalisation
template <typename IntType>
void
rational<IntType>::normalize() {
    // Avoid repeated construction
    IntType zero(0);

    if (den == zero)
        throw bad_rational();

    // Handle the case of zero separately, to avoid division by zero
    if (num == zero) {
        den = IntType(1);
        return;
    }

    IntType g = gcd(num, den);

    num /= g;
    den /= g;

    if (den < -(std::numeric_limits<IntType>::max)()) {
        throw bad_rational("bad rational: non-zero singular denominator");
    }

    // Ensure that the denominator is positive
    if (den < zero) {
        num = -num;
        den = -den;
    }

    assert(this->test_invariant());
}

namespace detail {

    // A utility class to reset the format flags for an istream at end
    // of scope, even in case of exceptions
    struct resetter {
        resetter(std::istream& is) : is_(is), f_(is.flags()) {}
        ~resetter() {
            is_.flags(f_);
        }
        std::istream& is_;
        std::istream::fmtflags f_; // old GNU c++ lib has no ios_base
    };

} // namespace detail

// Input and output
template <typename IntType>
std::istream&
operator>>(std::istream& is, rational<IntType>& r) {
    using std::ios;

    IntType n = IntType(0), d = IntType(1);
    char c = 0;
    detail::resetter sentry(is);

    if (is >> n) {
        if (is.get(c)) {
            if (c == '/') {
                if (is >> std::noskipws >> d)
                    try {
                        r.assign(n, d);
                    } catch (bad_rational&) { // normalization fail
                        try {
                            is.setstate(ios::failbit);
                        } catch (...) {
                        } // don't throw ios_base::failure...
                        if (is.exceptions() & ios::failbit)
                            throw; // ...but the original exception instead
                        // ELSE: suppress the exception, use just error flags
                    }
            } else
                is.setstate(ios::failbit);
        }
    }

    return is;
}

// Add manipulators for output format?
template <typename IntType>
std::ostream&
operator<<(std::ostream& os, const rational<IntType>& r) {
    // The slash directly precedes the denominator, which has no prefixes.
    std::ostringstream ss;

    ss.copyfmt(os);
    ss.tie(NULL);
    ss.exceptions(std::ios::goodbit);
    ss.width(0);
    ss << std::noshowpos << std::noshowbase << '/' << r.denominator();

    // The numerator holds the showpos, internal, and showbase flags.
    std::string const tail = ss.str();
    std::streamsize const w =
        os.width() - static_cast<std::streamsize>(tail.size());

    ss.clear();
    ss.str("");
    ss.flags(os.flags());
    ss << std::setw(
              w < 0 ||
                      (os.flags() & std::ios::adjustfield) != std::ios::internal
                  ? 0
                  : w)
       << r.numerator();
    return os << ss.str() + tail;
}

// Type conversion
template <typename T, typename IntType>
T
rational_cast(const rational<IntType>& src) {
    return static_cast<T>(src.numerator()) / static_cast<T>(src.denominator());
}

template <typename IntType>
rational<IntType>
abs(const rational<IntType>& r) {
    return r.numerator() >= IntType(0) ? r : -r;
}

} // namespace rationals
