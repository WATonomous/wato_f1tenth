#include "clothoid_g2.hpp"

#include <algorithm>
#include <cfloat>
#include <cmath>

namespace local_planning
{
namespace clothoid
{
namespace
{

using std::abs;
using std::max;
using std::min;
using std::fpclassify;

using real_type = double;
using integer = int;

// Upstream reads these from its Utils library; they are just constants.
constexpr real_type m_pi   = 3.141592653589793238462643383279502884197;
constexpr real_type m_2pi  = 6.283185307179586476925286766559005768394;
constexpr real_type m_pi_2 = 1.570796326794896619231321691639751442098;
constexpr real_type m_1_pi = 0.3183098861837906715377675267450287240689;
constexpr real_type m_1_sqrt_pi = 0.564189583547756286948079451561;

// Fresnel series/asymptotic switchover, upstream's values.
constexpr real_type A_THRESOLD = 0.01;
constexpr integer   A_SERIE_SIZE = 3;

inline bool is_zero(real_type const x) { return FP_ZERO == fpclassify(x); }

inline real_type power2(real_type const a) {return a * a;}
inline real_type power3(real_type const a) {return a * a * a;}
inline real_type power4(real_type const a) {real_type const a2 = a * a; return a2 * a2;}

void
rangeSymm( real_type & ang ) {
  ang = fmod( ang, m_2pi );
  while ( ang < -m_pi ) ang += m_2pi;
  while ( ang >  m_pi ) ang -= m_2pi;
}

// Upstream's 2x2 solve with full pivoting, used for the Newton step.
class Solve2x2 {
  integer   i[2]{0,0};
  integer   j[2]{0,0};
  real_type LU[2][2]{{0,0},{0,0}};
  real_type epsi{1e-10};
  bool      singular{false};
public:
  Solve2x2() = default;
  bool factorize( real_type A[2][2] );
  bool solve( real_type const b[2], real_type x[2] ) const;
};

  static constexpr real_type fn[] = {
    0.49999988085884732562,
    1.3511177791210715095,
    1.3175407836168659241,
    1.1861149300293854992,
    0.7709627298888346769,
    0.4173874338787963957,
    0.19044202705272903923,
    0.06655998896627697537,
    0.022789258616785717418,
    0.0040116689358507943804,
    0.0012192036851249883877
  };

  static constexpr real_type fd[] = {
    1.0,
    2.7022305772400260215,
    4.2059268151438492767,
    4.5221882840107715516,
    3.7240352281630359588,
    2.4589286254678152943,
    1.3125491629443702962,
    0.5997685720120932908,
    0.20907680750378849485,
    0.07159621634657901433,
    0.012602969513793714191,
    0.0038302423512931250065
  };

  static constexpr real_type gn[] = {
    0.50000014392706344801,
    0.032346434925349128728,
    0.17619325157863254363,
    0.038606273170706486252,
    0.023693692309257725361,
    0.007092018516845033662,
    0.0012492123212412087428,
    0.00044023040894778468486,
    -8.80266827476172521e-6,
    -1.4033554916580018648e-8,
    2.3509221782155474353e-10
  };

  static constexpr real_type gd[] = {
    1.0,
    2.0646987497019598937,
    2.9109311766948031235,
    2.6561936751333032911,
    2.0195563983177268073,
    1.1167891129189363902,
    0.57267874755973172715,
    0.19408481169593070798,
    0.07634808341431248904,
    0.011573247407207865977,
    0.0044099273693067311209,
    -0.00009070958410429993314
  };

  void
  FresnelCS( real_type y, real_type & C, real_type & S ) {

    constexpr real_type eps { 1E-15 };
    real_type const x{ y > 0 ? y : -y };

    if ( x < 1.0 ) {
      real_type term;

      real_type const s { m_pi_2*(x*x) };
      real_type const t { -s*s };

      // Cosine integral series
      real_type twofn   { 0.0 };
      real_type fact    { 1.0 };
      real_type denterm { 1.0 };
      real_type numterm { 1.0 };
      real_type sum     { 1.0 };
      do {
        twofn   += 2.0;
        fact    *= twofn*(twofn-1.0);
        denterm += 4.0;
        numterm *= t;
        term     = numterm/(fact*denterm);
        sum     += term;
      } while ( abs(term) > eps*abs(sum) );

      C = x*sum;

      // Sine integral series
      twofn   = 1.0;
      fact    = 1.0;
      denterm = 3.0;
      numterm = 1.0;
      sum     = 1.0/3.0;
      do {
        twofn   += 2.0;
        fact    *= twofn*(twofn-1.0);
        denterm += 4.0;
        numterm *= t;
        term     = numterm/(fact*denterm);
        sum     += term;
      } while ( abs(term) > eps*abs(sum) );

      S = m_pi_2*sum*(x*x*x);

    } else if ( x < 6.0 ) {

      // Rational approximation for f
      real_type sumn{ 0.0 };
      real_type sumd{ fd[11] };
      for ( integer k=10; k >= 0; --k ) {
        sumn = fn[k] + x*sumn;
        sumd = fd[k] + x*sumd;
      }
      real_type const f{ sumn/sumd };

      // Rational approximation for g
      sumn = 0.0;
      sumd = gd[11];
      for ( integer k=10; k >= 0; --k ) {
        sumn = gn[k] + x*sumn;
        sumd = gd[k] + x*sumd;
      }
      real_type const g    { sumn/sumd };
      real_type const U    { m_pi_2*(x*x) };
      real_type const SinU { sin(U) };
      real_type const CosU { cos(U) };
      C = 0.5 + f*SinU - g*CosU;
      S = 0.5 - f*CosU - g*SinU;

    } else {

      real_type absterm;

      // x >= 6; asymptotic expansions for  f  and  g

      real_type const s { m_pi*x*x };
      real_type const t { -1/(s*s) };

      // Expansion for f
      real_type       numterm {-1.0 };
      real_type       term    { 1.0 };
      real_type       sum     { 1.0 };
      real_type       oldterm { 1.0 };
      real_type const eps10   { 0.1 * eps };

      do {
        numterm += 4.0;
        term    *= numterm*(numterm-2.0)*t;
        sum     += term;
        absterm  = abs(term);
        oldterm  = absterm;
      } while ( absterm > eps10 * abs(sum) );

      real_type const f{ sum / (m_pi*x) };

      //  Expansion for  g
      numterm = -1.0;
      term    =  1.0;
      sum     =  1.0;
      oldterm =  1.0;

      do {
        numterm += 4.0;
        term    *= numterm*(numterm+2.0)*t;
        sum     += term;
        absterm  = abs(term);
        oldterm  = absterm;
      } while ( absterm > eps10 * abs(sum) );

      real_type       g    { m_pi*x }; g = sum/(g*g*x);
      real_type const U    { m_pi_2*(x*x) };
      real_type const SinU { sin(U) };
      real_type const CosU { cos(U) };
      C = 0.5 + f*SinU - g*CosU;
      S = 0.5 - f*CosU - g*SinU;

    }
    if ( y < 0 ) { C = -C; S = -S; }
  }

  void
  FresnelCS(
    integer   const nk,
    real_type const t,
    real_type       C[],
    real_type       S[]
  ) {
    FresnelCS(t,C[0],S[0]);
    if ( nk > 1 ) {
      real_type const tt { m_pi_2*(t*t) };
      real_type const ss { sin(tt) };
      real_type const cc { cos(tt) };
      C[1] = ss*m_1_pi;
      S[1] = (1-cc)*m_1_pi;
      if ( nk > 2 ) {
        C[2] = (t*ss-S[0])*m_1_pi;
        S[2] = (C[0]-t*cc)*m_1_pi;
      }
    }
  }

  static
  void
  evalXYaLarge(
    real_type const a,
    real_type const b,
    real_type &     X,
    real_type &     Y
  ) {
    real_type const s    = a > 0 ? +1 : -1;
    real_type const absa = abs(a);
    real_type const z    = m_1_sqrt_pi*sqrt(absa);
    real_type const ell  = s*b*m_1_sqrt_pi/sqrt(absa);
    real_type const g    = -0.5*s*(b*b)/absa;
    real_type const cg   = cos(g)/z;
    real_type const sg   = sin(g)/z;

    real_type Cl, Sl, Cz, Sz;
    FresnelCS( ell,   Cl, Sl );
    FresnelCS( ell+z, Cz, Sz );

    real_type const dC0{ Cz - Cl };
    real_type const dS0{ Sz - Sl };

    X = cg * dC0 - s * sg * dS0;
    Y = sg * dC0 + s * cg * dS0;
  }

  static
  void
  evalXYaLarge(
    integer   const nk,
    real_type const a,
    real_type const b,
    real_type       X[],
    real_type       Y[]
  ) {


    real_type const s    { static_cast<real_type>(a > 0 ? +1 : -1) };
    real_type const absa { abs(a) };
    real_type const z    { m_1_sqrt_pi*sqrt(absa) };
    real_type const ell  { s*b*m_1_sqrt_pi/sqrt(absa) };
    real_type const g    { -0.5*s*(b*b)/absa };
    real_type       cg   { cos(g)/z };
    real_type       sg   { sin(g)/z };

    real_type Cl[3], Sl[3], Cz[3], Sz[3];

    FresnelCS( nk, ell,   Cl, Sl );
    FresnelCS( nk, ell+z, Cz, Sz );

    real_type const dC0 { Cz[0] - Cl[0] };
    real_type const dS0 { Sz[0] - Sl[0] };
    X[0] = cg * dC0 - s * sg * dS0;
    Y[0] = sg * dC0 + s * cg * dS0;
    if ( nk > 1 ) {
      cg /= z;
      sg /= z;
      real_type const dC1 { Cz[1] - Cl[1] };
      real_type const dS1 { Sz[1] - Sl[1] };
      real_type       DC  { dC1-ell*dC0   };
      real_type       DS  { dS1-ell*dS0   };
      X[1] = cg * DC - s * sg * DS;
      Y[1] = sg * DC + s * cg * DS;
      if ( nk > 2 ) {
        real_type const dC2{ Cz[2] - Cl[2] };
        real_type const dS2{ Sz[2] - Sl[2] };
        DC   = dC2+ell*(ell*dC0-2*dC1);
        DS   = dS2+ell*(ell*dS0-2*dS1);
        cg   = cg/z;
        sg   = sg/z;
        X[2] = cg * DC - s * sg * DS;
        Y[2] = sg * DC + s * cg * DS;
      }
    }
  }

  static
  real_type
  LommelReduced( real_type const mu, real_type const nu, real_type const b ) {
    real_type tmp{ 1/((mu+nu+1)*(mu-nu+1)) };
    real_type res{ tmp };
    for ( integer n = 1; n <= 100; ++n ) {
      tmp *= (-b/(2*n+mu-nu+1)) * (b/(2*n+mu+nu+1));
      res += tmp;
      if ( abs(tmp) < abs(res) * 1e-50 ) break;
    }
    return res;
  }

  static
  void
  evalXYazero(
    integer   const nk,
    real_type const b,
    real_type       X[],
    real_type       Y[]
  ) {
    real_type const sb{ sin(b) };
    real_type const cb{ cos(b) };
    real_type const b2{ b*b };
    if ( abs(b) < 1e-3 ) {
      X[0] = 1-(b2/6)*(1-(b2/20)*(1-(b2/42)));
      Y[0] = (b/2)*(1-(b2/12)*(1-(b2/30)));
    } else {
      X[0] = sb/b;
      Y[0] = (1-cb)/b;
    }
    // use recurrence in the stable part
    integer m{ static_cast<integer>(floor(2 * b)) };
    if ( m >= nk ) m = nk-1;
    if ( m < 1   ) m = 1;
    for ( integer k{1}; k < m; ++k ) {
      X[k] = (sb-k*Y[k-1])/b;
      Y[k] = (k*X[k-1]-cb)/b;
    }
    //  use Lommel for the unstable part
    if ( m < nk ) {
      real_type const A   { b*sb    };
      real_type const D   { sb-b*cb };
      real_type const B   { b*D     };
      real_type const C   { -b2*sb  };
      real_type       rLa { LommelReduced(m+0.5,1.5,b) };
      real_type       rLd { LommelReduced(m+0.5,0.5,b) };
      for ( integer k{m}; k < nk; ++k ) {
        real_type const rLb { LommelReduced(k+1.5,0.5,b) };
        real_type const rLc { LommelReduced(k+1.5,1.5,b) };
        X[k] = ( k*A*rLa + B*rLb + cb ) / (1+k);
        Y[k] = ( C*rLc + sb ) / (2+k) + D*rLd;
	      rLa  = rLc;
  	    rLd  = rLb;
      }
    }
  }

  static
  void
  evalXYaSmall(
    real_type const a,
    real_type const b,
    integer   const p,
    real_type &     X,
    real_type &     Y
  ) {


    real_type X0[43], Y0[43];

    integer const nkk{ 4*p + 3 }; // max 43
    evalXYazero( nkk, b, X0, Y0 );

    X = X0[0]-(a/2)*Y0[2];
    Y = Y0[0]+(a/2)*X0[2];

    real_type       t  { 1 };
    real_type const aa { -a*a/4 }; // controllare!
    for ( integer n{1}; n <= p; ++n ) {
      t *= aa/(2*n*(2*n-1));
      real_type const bf{ a/(4*n+2) };
      integer   const jj{ 4*n };
      X += t*(X0[jj]-bf*Y0[jj+2]);
      Y += t*(Y0[jj]+bf*X0[jj+2]);
    }
  }

  static
  void
  evalXYaSmall(
    integer   const nk,
    real_type const a,
    real_type const b,
    integer   const p,
    real_type       X[],
    real_type       Y[]
  ) {

    integer   nkk{nk + 4*p + 2}; // max 45
    real_type X0[45], Y0[45];


    evalXYazero( nkk, b, X0, Y0 );

    for ( integer j=0; j < nk; ++j ) {
      X[j] = X0[j]-(a/2)*Y0[j+2];
      Y[j] = Y0[j]+(a/2)*X0[j+2];
    }

    real_type       t  { 1 };
    real_type const aa { -a*a/4 }; // controllare!
    for ( integer n{1}; n <= p; ++n ) {
      t *= aa/(2*n*(2*n-1));
      real_type const bf{ a/(4*n+2) };
      for ( integer j{0}; j < nk; ++j ) {
        integer const jj{ 4*n+j };
        X[j] += t*(X0[jj]-bf*Y0[jj+2]);
        Y[j] += t*(Y0[jj]+bf*X0[jj+2]);
      }
    }
  }

  void
  GeneralizedFresnelCS(
    real_type const a,
    real_type const b,
    real_type const c,
    real_type &     intC,
    real_type &     intS
  ) {
    real_type xx, yy;
    if ( abs(a) < A_THRESOLD ) evalXYaSmall( a, b, A_SERIE_SIZE, xx, yy );
    else                       evalXYaLarge( a, b, xx, yy );

    real_type const cosc{ cos(c) };
    real_type const sinc{ sin(c) };

    intC = xx * cosc - yy * sinc;
    intS = xx * sinc + yy * cosc;
  }

  void
  GeneralizedFresnelCS(
    integer   const nk,
    real_type const a,
    real_type const b,
    real_type const c,
    real_type       intC[],
    real_type       intS[]
  ) {

    if ( abs(a) < A_THRESOLD ) evalXYaSmall( nk, a, b, A_SERIE_SIZE, intC, intS );
    else                       evalXYaLarge( nk, a, b, intC, intS );

    real_type const cosc{ cos(c) };
    real_type const sinc{ sin(c) };

    for ( integer k{0}; k < nk; ++k ) {
      real_type const xx{ intC[k] };
      real_type const yy{ intS[k] };
      intC[k] = xx * cosc - yy * sinc;
      intS[k] = xx * sinc + yy * cosc;
    }
  }


// Upstream's ClothoidData plus the arc length that ClothoidCurve used to hold.
// Only the members the G2 solve and its G1 initial guess actually touch.
class CData {
public:
  real_type m_x0{0}, m_y0{0}, m_theta0{0}, m_kappa0{0}, m_dk{0}, m_L{0};

  real_type theta( real_type const s ) const { return m_theta0 + s*(m_kappa0 + 0.5*m_dk*s); }
  real_type kappa( real_type const s ) const { return m_kappa0 + s*m_dk; }
  real_type theta_begin() const { return m_theta0; }
  real_type theta_end()   const { return theta(m_L); }
  real_type kappa_begin() const { return m_kappa0; }
  real_type kappa_end()   const { return kappa(m_L); }
  real_type dkappa()      const { return m_dk; }
  real_type length()      const { return m_L; }

  void
  build( real_type const x0, real_type const y0, real_type const theta0,
         real_type const kappa0, real_type const dk, real_type const L ) {
    m_x0 = x0; m_y0 = y0; m_theta0 = theta0; m_kappa0 = kappa0; m_dk = dk; m_L = L;
  }

  bool
  build_G1( real_type const x0, real_type const y0, real_type const theta0,
            real_type const x1, real_type const y1, real_type const theta1,
            real_type const tol = 1e-12 ) {
    return build_G1_impl( x0, y0, theta0, x1, y1, theta1, tol, m_L ) >= 0;
  }

  // Moves the arc's parametric origin to s_origin, upstream's origin_at.
  void
  origin_at( real_type const s_origin ) {
    real_type C, S;
    real_type const sdk = s_origin*m_dk;
    GeneralizedFresnelCS( sdk*s_origin, m_kappa0*s_origin, m_theta0, C, S );
    m_x0     += s_origin*C;
    m_y0     += s_origin*S;
    m_theta0 += s_origin*(m_kappa0+0.5*sdk);
    m_kappa0 += sdk;
  }

  void
  change_curvilinear_origin( real_type const s0, real_type const newL ) {
    origin_at( s0 );
    m_L = newL;
  }

  int
  build_G1_impl( real_type _x0, real_type _y0, real_type _theta0,
                 real_type x1, real_type y1, real_type theta1,
                 real_type tol, real_type & L );
};

// Solves the three-arc G2 problem.  Upstream's G2solve3arc with the curve
// objects replaced by CData and the error macros replaced by a bool.
class Solver {
public:
  CData m_S0, m_SM, m_S1;

  real_type m_tolerance{1e-10};
  int       m_max_iter{100};

  real_type m_x0{0}, m_y0{0}, m_theta0{0}, m_kappa0{0};
  real_type m_x1{0}, m_y1{0}, m_theta1{0}, m_kappa1{0};
  real_type m_phi{0}, m_Lscale{0}, m_th0{0}, m_th1{0}, m_s0{0}, m_s1{0};
  real_type m_K0{0}, m_K1{0}, m_c0{0}, m_c1{0}, m_c2{0}, m_c3{0}, m_c4{0},
            m_c5{0}, m_c6{0}, m_c7{0}, m_c8{0}, m_c9{0}, m_c10{0}, m_c11{0},
            m_c12{0}, m_c13{0}, m_c14{0};

  void evalFJ( real_type const vars[2], real_type F[2], real_type J[2][2] ) const;
  void evalF( real_type const vars[2], real_type F[2] ) const;
  void build_solution( real_type sM, real_type thM );
  int  solve( real_type sM_guess, real_type thM_guess );
  int  build( real_type x0, real_type y0, real_type theta0, real_type kappa0,
              real_type x1, real_type y1, real_type theta1, real_type kappa1,
              real_type Dmax = 0, real_type dmax = 0 );
};

  bool
  Solve2x2::factorize( real_type A[2][2] ) {
    // full pivoting
    real_type Amax = abs(A[0][0]);
    real_type tmp  = abs(A[0][1]);
    integer   ij{0};
    if ( tmp > Amax ) { ij = 1; Amax = tmp; }
    tmp = abs(A[1][0]);
    if ( tmp > Amax ) { ij = 2; Amax = tmp; }
    tmp = abs(A[1][1]);
    if ( tmp > Amax ) { ij = 3; Amax = tmp; }
    if ( is_zero(Amax) ) return false;
    if ( (ij&0x01) == 0x01 ) { j[0] = 1; j[1] = 0; }
    else                     { j[0] = 0; j[1] = 1; }
    if ( (ij&0x02) == 0x02 ) { i[0] = 1; i[1] = 0; }
    else                     { i[0] = 0; i[1] = 1; }
    // apply factorization
    LU[0][0] = A[i[0]][j[0]];
    LU[0][1] = A[i[0]][j[1]];
    LU[1][0] = A[i[1]][j[0]];
    LU[1][1] = A[i[1]][j[1]];

    LU[1][0] /= LU[0][0];
    LU[1][1] -= LU[1][0]*LU[0][1];
    // check for singularity
    singular = abs( LU[1][1] ) < epsi;
    return true;
  }

  bool
  Solve2x2::solve( real_type const b[2], real_type x[2] ) const {
    if ( singular ) {
      // L^+ Pb
      real_type tmp = (b[i[0]] + LU[1][0]*b[i[1]]) /
                      ( (1+power2(LU[1][0]) ) * ( power2(LU[0][0])+power2(LU[0][1]) ) );
      x[j[0]] = tmp*LU[0][0];
      x[j[1]] = tmp*LU[0][1];
      // check consistency
      tmp = (LU[0][0]*x[j[0]]+LU[0][1]*x[j[1]]);
      return hypot( b[i[0]]-tmp, b[i[1]]+tmp*LU[1][0] ) < hypot(b[0],b[1])*epsi;
    }
    // non singular
    // L^(-1) Pb
    x[j[0]] = b[i[0]];
    x[j[1]] = b[i[1]]-LU[1][0]*x[j[0]];
    // U^(-1) x
    x[j[1]] /= LU[1][1];
    x[j[0]]  = (x[j[0]]-LU[0][1]*x[j[1]])/LU[0][0];
    return FP_INFINITE != fpclassify(x[0]) &&
           FP_NAN      != fpclassify(x[0]) &&
           FP_INFINITE != fpclassify(x[1]) &&
           FP_NAN      != fpclassify(x[1]);
  }

  int
  CData::build_G1_impl(
    real_type const _x0,
    real_type const _y0,
    real_type const _theta0,
    real_type const x1,
    real_type const y1,
    real_type const theta1,
    real_type const tol,
    real_type &     L
  ) {
    constexpr bool compute_deriv{false};
    real_type L_D[2]{0,0}, k_D[2]{0,0}, dk_D[2]{0,0};
    (void)L_D; (void)k_D; (void)dk_D;
    static constexpr real_type CF[]{
      2.989696028701907,   0.716228953608281,
      -0.458969738821509, -0.502821153340377,
      0.261062141752652,  -0.045854475238709
    };

    m_x0     = _x0;
    m_y0     = _y0;
    m_theta0 = _theta0;

    // traslazione in (0,0)
    real_type const dx   = x1 - m_x0;
    real_type const dy   = y1 - m_y0;
    real_type const r    = hypot( dx, dy );
    real_type const phi  = atan2( dy, dx );
    real_type       phi0 = m_theta0 - phi;
    real_type       phi1 = theta1 - phi;

    phi0 -= m_2pi*round(phi0/m_2pi);
    phi1 -= m_2pi*round(phi1/m_2pi);

    if      ( phi0 >  m_pi ) phi0 -= m_2pi;
    else if ( phi0 < -m_pi ) phi0 += m_2pi;
    if      ( phi1 >  m_pi ) phi1 -= m_2pi;
    else if ( phi1 < -m_pi ) phi1 += m_2pi;

    real_type delta = phi1 - phi0;

    // punto iniziale
    real_type       X  { phi0*m_1_pi };
    real_type       Y  { phi1*m_1_pi };
    real_type const xy { X*Y };
    Y *= Y; X *= X;
    real_type A{ (phi0+phi1) * ( CF[0] + xy*(CF[1] + xy*CF[2]) +
                               ( CF[3]+xy*CF[4])*(X+Y) + CF[5]*(X*X+Y*Y) ) };
    // newton
    real_type g{0}, intC[3], intS[3];
    integer   niter{0};
    do {
      GeneralizedFresnelCS( 3, 2*A, delta-A, phi0, intC, intS );
      g   = intS[0];
      real_type const dg{ intC[2] - intC[1] };
      A  -= g / dg;
    } while ( ++niter <= 10 && abs(g) > tol );

    GeneralizedFresnelCS( 2*A, delta-A, phi0, intC[0], intS[0] );
    L = r/intC[0];

    m_kappa0 = (delta-A)/L;
    m_dk     = 2*A/L/L;

    if ( compute_deriv ) {

      real_type const alpha { intC[0]*intC[1] + intS[0]*intS[1] };
      real_type const beta  { intC[0]*intC[2] + intS[0]*intS[2] };
      real_type const gamma { intC[0]*intC[0] + intS[0]*intS[0] };
      real_type const tx    { intC[1]-intC[2] };
      real_type const ty    { intS[1]-intS[2] };
      real_type const txy   { L*(intC[1]*intS[2]-intC[2]*intS[1]) };
      real_type const omega { L*(intS[0]*tx-intC[0]*ty) - txy };

      delta = intC[0]*tx + intS[0]*ty;

      L_D[0] = omega/delta;
      L_D[1] = txy/delta;

      delta *= L;
      k_D[0] = (beta-gamma-m_kappa0*omega)/delta;
      k_D[1] = -(beta+m_kappa0*txy)/delta;

      delta  *= L/2;
      dk_D[0] = (gamma-alpha-m_dk*omega*L)/delta;
      dk_D[1] = (alpha-m_dk*txy*L)/delta;
    }

    return niter;
  }

  int
  Solver::build(
    real_type const x0,
    real_type const y0,
    real_type const theta0,
    real_type const kappa0,
    real_type const x1,
    real_type const y1,
    real_type const theta1,
    real_type const kappa1,
    real_type       Dmax,
    real_type       dmax
  ) {
    try {
      // save data
      m_x0     = x0;
      m_y0     = y0;
      m_theta0 = theta0;
      m_kappa0 = kappa0;
      m_x1     = x1;
      m_y1     = y1;
      m_theta1 = theta1;
      m_kappa1 = kappa1;

      // transform to reference frame
      real_type const dx{m_x1 - m_x0};
      real_type const dy{m_y1 - m_y0};
      m_phi    = atan2( dy, dx );
      m_Lscale = 2/hypot( dx, dy );

      m_th0 = m_theta0 - m_phi;
      m_th1 = m_theta1 - m_phi;

      // put in range
      rangeSymm(m_th0);
      rangeSymm(m_th1);

      m_K0 = (m_kappa0/m_Lscale); // k0
      m_K1 = (m_kappa1/m_Lscale); // k1

      if ( Dmax <= 0 ) Dmax = m_pi;
      if ( dmax <= 0 ) dmax = m_pi/8;

      if ( Dmax > m_2pi  ) Dmax = m_2pi;
      if ( dmax > m_pi/4 ) dmax = m_pi/4;

      // compute guess G1
      CData SG;
      SG.build_G1( -1, 0, m_th0, 1, 0, m_th1 );

      real_type const kA { SG.kappa_begin() };
      real_type const kB { SG.kappa_end() };
      real_type const dk { abs(SG.dkappa()) };
      real_type const L3 { SG.length()/3 };

      real_type tmp { 0.5*abs(m_K0-kA)/dmax };
      m_s0 = L3;
      if ( tmp*m_s0 > 1 ) m_s0 = 1/tmp;
      tmp = (abs(m_K0+kA)+m_s0*dk)/(2*Dmax);
      if ( tmp*m_s0 > 1 ) m_s0 = 1/tmp;

      tmp = 0.5*abs(m_K1-kB)/dmax;
      m_s1 = L3;
      if ( tmp*m_s1 > 1 ) m_s1 = 1/tmp;
      tmp = (abs(m_K1+kB)+m_s1*dk)/(2*Dmax);
      if ( tmp*m_s1 > 1 ) m_s1 = 1/tmp;

      real_type const dth   { abs(m_th0-m_th1) / m_2pi };
      real_type const scale { power3(cos( power4(dth)*m_pi_2 )) };
      m_s0 *= scale;
      m_s1 *= scale;

      real_type const L   { (3*L3-m_s0-m_s1)/2 };
      real_type const thM { SG.theta(m_s0+L) };
      m_th0 = SG.theta_begin();
      m_th1 = SG.theta_end();

      // setup

      m_K0 *= m_s0;
      m_K1 *= m_s1;

      real_type const t0 { 2*m_th0+m_K0 };
      real_type const t1 { 2*m_th1-m_K1 };

      m_c0  = m_s0*m_s1;
      m_c1  = 2 * m_s0;
      m_c2  = 0.25*((m_K1-6*(m_K0+m_th0)-2*m_th1)*m_s0 - 3*m_K0*m_s1);
      m_c3  = -m_c0 * (m_K0 + m_th0);
      m_c4  = 2 * m_s1;
      m_c5  = 0.25*((6*(m_K1-m_th1)-m_K0-2*m_th0)*m_s1 + 3*m_K1*m_s0);
      m_c6  = m_c0 * (m_K1 - m_th1);
      m_c7  = -0.5*(m_s0 + m_s1);
      m_c8  = m_th0 + m_th1 + 0.5*(m_K0 - m_K1);
      m_c9  = 0.25*(t1*m_s0 + t0*m_s1);
      m_c10 = 0.5*(m_s1 - m_s0);
      m_c11 = 0.5*(m_th1 - m_th0) - 0.25*(m_K0 + m_K1);
      m_c12 = 0.25*(t1*m_s0 - t0*m_s1);
      m_c13 = 0.5*m_s0*m_s1;
      m_c14 = 0.75*(m_s0 + m_s1);
      return solve( L, thM );
    } catch (...) {
      return -1;
      // nothing to do
    }
  }

  void
  Solver::evalFJ(
    real_type const vars[2],
    real_type       F[2],
    real_type       J[2][2]
  ) const {

    real_type const sM  { vars[0] };
    real_type const thM { vars[1] };

    real_type const dsM   { 1.0 / (m_c13+(m_c14+sM)*sM) };
    real_type const dsMsM { dsM*sM };
    real_type const dK0   { dsM*(m_c0*thM + sM*(m_c1*thM + m_c2 - sM*m_K0) + m_c3) };
    real_type const dK1   { dsM*(m_c0*thM + sM*(m_c4*thM + m_c5 + sM*m_K1) + m_c6) };
    real_type const dKM   { dsMsM*(thM*(m_c7-2*sM) + m_c8*sM + m_c9) };
    real_type const KM    { dsMsM*(m_c10*thM + m_c11*sM + m_c12) };

    real_type X0[3],  Y0[3],
              X1[3],  Y1[3],
              XMp[3], YMp[3],
              XMm[3], YMm[3];
    GeneralizedFresnelCS( 3, dK0,  m_K0, m_th0, X0,  Y0);
    GeneralizedFresnelCS( 3, dK1, -m_K1, m_th1, X1,  Y1);
    GeneralizedFresnelCS( 3, dKM,    KM,   thM, XMp, YMp);
    GeneralizedFresnelCS( 3, dKM,   -KM,   thM, XMm, YMm);

    // in the standard problem dx = 2, dy = 0
    real_type const t0 { XMp[0]+XMm[0] };
    real_type const t1 { YMp[0]+YMm[0] };
    F[0] = m_s0*X0[0] + m_s1*X1[0] + sM*t0 - 2;
    F[1] = m_s0*Y0[0] + m_s1*Y1[0] + sM*t1 - 0;

    // calcolo J(F)
    real_type const dsM2 { dsM*dsM };
    real_type const g0   { -(2 * sM + m_c14)*dsM2 };
    real_type const g1   { (m_c13 - sM*sM)*dsM2 };
    real_type const g2   { sM*(sM*m_c14+2*m_c13)*dsM2 };

    real_type const dK0_sM  { (m_c0*thM+m_c3)*g0 + (m_c1*thM+m_c2)*g1 - m_K0*g2 };
    real_type const dK1_sM  { (m_c0*thM+m_c6)*g0 + (m_c4*thM+m_c5)*g1 + m_K1*g2 };
    real_type const dKM_sM  { (m_c7*thM+m_c9)*g1 + (m_c8-2*thM)*g2 };
    real_type const KM_sM   { (m_c10*thM+m_c12)*g1 + m_c11*g2 };

    real_type const dK0_thM { (m_c0+m_c1*sM)*dsM };
    real_type const dK1_thM { (m_c0+m_c4*sM)*dsM };
    real_type const dKM_thM { (m_c7-2*sM)*dsMsM };
    real_type const KM_thM  { m_c10*dsMsM };

    // coeff fresnel per f_j per lo jacobiano
    real_type const f0 { -0.5*m_s0*Y0[2] };
    real_type const f1 { -0.5*m_s1*Y1[2] };
    real_type const f2 { -0.5*sM*(YMm[2] + YMp[2]) };
    real_type const f3 { sM*(YMm[1] - YMp[1]) };
    real_type const f4 { 0.5*m_s0*X0[2] };
    real_type const f5 { 0.5*m_s1*X1[2] };
    real_type const f6 { 0.5*sM*(XMm[2] + XMp[2]) };
    real_type const f7 { sM*(XMp[1] - XMm[1]) };

    J[0][0] = f0 * dK0_sM  + f1 * dK1_sM  + f2 * dKM_sM  + f3 * KM_sM  + t0;
    J[0][1] = f0 * dK0_thM + f1 * dK1_thM + f2 * dKM_thM + f3 * KM_thM - sM * t1;
    J[1][0] = f4 * dK0_sM  + f5 * dK1_sM  + f6 * dKM_sM  + f7 * KM_sM  + t1;
    J[1][1] = f4 * dK0_thM + f5 * dK1_thM + f6 * dKM_thM + f7 * KM_thM + sM * t0;
  }

  void
  Solver::evalF( real_type const vars[2], real_type F[2] ) const {

    real_type const sM  { vars[0] };
    real_type const thM { vars[1] };

    real_type const dsM { 1.0 / (m_c13+(m_c14+sM)*sM) };
    real_type const dK0 { dsM*(m_c0*thM + sM*(m_c1*thM - m_K0*sM + m_c2) + m_c3) };
    real_type const dK1 { dsM*(m_c0*thM + sM*(m_c4*thM + m_K1*sM + m_c5) + m_c6) };
    real_type const dKM { dsM*sM*( thM*(m_c7-2*sM) + m_c8*sM + m_c9) };
    real_type const KM  { dsM*sM*(m_c10*thM + m_c11*sM + m_c12) };

    real_type X0, Y0, X1, Y1, XMp, YMp, XMm, YMm;
    GeneralizedFresnelCS( dK0,  m_K0, m_th0, X0,  Y0);
    GeneralizedFresnelCS( dK1, -m_K1, m_th1, X1,  Y1);
    GeneralizedFresnelCS( dKM,  KM,   thM,   XMp, YMp);
    GeneralizedFresnelCS( dKM, -KM,   thM,   XMm, YMm);

    // in the standard problem dx = 2, dy = 0
    F[0] = m_s0*X0 + m_s1*X1 + sM*(XMm + XMp) - 2;
    F[1] = m_s0*Y0 + m_s1*Y1 + sM*(YMm + YMp) - 0;
  }

  void
  Solver::build_solution( real_type const sM, real_type const thM ) {
    // soluzione nel frame di riferimento
    /* real_type k0 = K0
     S0.build( -1, 0, th0, k0, dK0,   0, L0 );
     S1.build( x1, y1, phi+th1, kappa1, dK1, -L1, 0  );
     S1.change_origin(-L1);
    */

    // ricostruzione dati clotoidi trasformati
    real_type const dsM { 1.0 / (m_c13+(m_c14+sM)*sM) };
    real_type       dK0 { dsM*(m_c0*thM + sM*(m_c1*thM - m_K0*sM + m_c2) + m_c3) };
    real_type       dK1 { dsM*(m_c0*thM + sM*(m_c4*thM + m_K1*sM + m_c5) + m_c6) };
    real_type       dKM { dsM*sM*(m_c7*thM + sM*(m_c8 - 2*thM) + m_c9) };
    real_type       KM  { dsM*sM*(m_c10*thM + m_c11*sM + m_c12) };

    real_type xa, ya, xmL, ymL;
    GeneralizedFresnelCS( dK0,  m_K0, m_th0, xa,  ya  );
    GeneralizedFresnelCS( dKM,   -KM,   thM, xmL, ymL );

    real_type const xM { m_s0 * xa + sM * xmL - 1 };
    real_type const yM { m_s0 * ya + sM * ymL };

    // rovescia trasformazione standard
    real_type const L0{ m_s0/m_Lscale };
    real_type const L1{ m_s1/m_Lscale };
    real_type const LM{ sM/m_Lscale   };

    dK0 *= power2(m_Lscale/m_s0);
    dK1 *= power2(m_Lscale/m_s1);
    dKM *= power2(m_Lscale/sM);
    KM  *= m_Lscale/sM;

    //th0 = theta0 - phi;
    //th1 = theta1 - phi;
    m_S0.build( m_x0, m_y0, m_phi+m_th0, m_kappa0, dK0, L0 );
    m_S1.build( m_x1, m_y1, m_phi+m_th1, m_kappa1, dK1, L1 );
    m_S1.change_curvilinear_origin( -L1, L1 );

    // la trasformazione inversa da [-1,1] a (x0,y0)-(x1,y1)
    // g(x,y) = RotInv(phi)*(1/lambda*[X;Y] - [xbar;ybar]) = [x;y]

    real_type const C  { cos(m_phi) };
    real_type const S  { sin(m_phi) };
    real_type const dx { (xM + 1) / m_Lscale };
    real_type const dy { yM / m_Lscale };
    m_SM.build(
      m_x0 + C * dx - S * dy,
      m_y0 + C * dy + S * dx,
      thM + m_phi, KM, dKM, 2*LM
    );
    m_SM.change_curvilinear_origin( -LM, 2*LM );
  }

  int
  Solver::solve( real_type const sM_guess, real_type const thM_guess ) {
    real_type X[2];
    X[0] = sM_guess;
    X[1] = thM_guess;

    //real_type thmin = min(th0,th1)-2*m_2pi;
    //real_type thmax = max(th0,th1)+2*m_2pi;

    integer iter{0};
    bool converged{false};
    try {
      Solve2x2 solver;
      do {
        real_type J[2][2];
        real_type d[2];
        real_type F[2];
        evalFJ(X, F, J);
        real_type const lenF{ hypot(F[0], F[1]) };
        converged = lenF < m_tolerance;
        if ( converged || !solver.factorize(J) ) break;
        solver.solve(F, d);
        #if 1
        // use undamped Newton
        X[0] -= d[0];
        X[1] -= d[1];
        #else
        real_type FF[2], dd[2], XX[2];
        // Affine invariant Newton solver
        real_type nd = hypot( d[0], d[1] );
        bool step_found = false;
        real_type tau = 2;
        do {
          tau  /= 2;
          XX[0] = X[0]-tau*d[0];
          XX[1] = X[1]-tau*d[1];
          evalF(XX, FF);
          solver.solve(FF, dd);
          step_found = hypot( dd[0], dd[1] ) <= (1-tau/2)*nd + 1e-6;
                       //&& XX[0] > 0; // && XX[0] > X[0]/4 && XX[0] < 4*X[0];
                       //&& XX[1] > thmin && XX[1] < thmax;
        } while ( tau > 1e-6 && !step_found );
        if ( !step_found ) break;
        X[0] = XX[0];
        X[1] = XX[1];
        #endif
      } while ( ++iter < m_max_iter );

      // re-check solution
      if ( converged )
        converged = FP_INFINITE != fpclassify(X[0]) &&
                    FP_NAN      != fpclassify(X[0]) &&
                    FP_INFINITE != fpclassify(X[1]) &&
                    FP_NAN      != fpclassify(X[1]);
    }
    catch (...) {
      // nothing to do
    }
    if ( converged ) build_solution(X[0], X[1]); // costruisco comunque soluzione
    return converged ? iter : -1;
  }

} // namespace

int solveG2(
  double x0, double y0, double theta0, double kappa0,
  double x1, double y1, double theta1, double kappa1,
  ThreeArcSolution & solution)
{
  Solver solver;
  const int iterations = solver.build(x0, y0, theta0, kappa0, x1, y1, theta1, kappa1);
  if (iterations < 0) {
    return -1;
  }

  const CData * const arcs[3] = {&solver.m_S0, &solver.m_SM, &solver.m_S1};
  for (int i = 0; i < 3; ++i) {
    solution.arcs[i].x0 = arcs[i]->m_x0;
    solution.arcs[i].y0 = arcs[i]->m_y0;
    solution.arcs[i].theta0 = arcs[i]->m_theta0;
    solution.arcs[i].kappa0 = arcs[i]->m_kappa0;
    solution.arcs[i].dk = arcs[i]->m_dk;
    solution.arcs[i].length = arcs[i]->m_L;
  }
  return iterations;
}

} // namespace clothoid
} // namespace local_planning
