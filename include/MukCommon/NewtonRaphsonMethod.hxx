
namespace gris
{
namespace numeric
{
  /**
  */
  template<class T, class J>
  bool NewtonRaphsonMethod<T,J>::solve(T& x, UsrFun usrfun, size_t ntrial, double tolx, double tolf)
  {
    const size_t N = x.size();
    T p, q, fvec;
    J fjac;    
    for (size_t k(0); k<ntrial; ++k)
    {
      usrfun(x, fvec, fjac);
      double errf = 0.0;
      for (size_t i(0); i<N; ++i)
      {
        errf += abs(fvec(i,0));
      }
      if (errf <= tolf)
      {
        return true;
      }
      for (size_t i(0); i<N; ++i)
      {
        p[i] = -fvec[i];
      }
      Eigen::FullPivLU<J> alu(fjac);
      q = alu.solve(p);
      double errx = 0.0;
      for (size_t i(0); i<N; ++i)
      {
        errx += abs(q[i]);
        x[i] += q[i];
      }
      if (errx <= tolx)
      {
        return true;
      }
    }
    return false;
  }
}
}
