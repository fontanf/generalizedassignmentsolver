#include "gap/lb_dip/dip.hpp"

#include "knapsack/opt_minknap/minknap.hpp"

#include "coin/VolVolume.hpp"
#include "coin/CoinHelperFunctions.hpp"
#include "coin/CoinPackedMatrix.hpp"

using namespace gap;

/**
 * Useful links to use Dip:
 * https://github.com/coin-or/Dip/blob/master/Dip/examples/GAP/GAP_DecompApp.cpp
 * http://coral.ie.lehigh.edu/~ted/files/talks/DecompCSIRO11.pdf
 * https://www.coin-or.org/Doxygen/Dip/class_decomp_app.html
 */

