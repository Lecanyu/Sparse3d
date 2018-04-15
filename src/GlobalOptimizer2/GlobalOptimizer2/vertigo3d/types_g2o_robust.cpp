#include <g2o/core/factory.h>
#include <g2o/stuff/macros.h>

#include "edge_switchPrior.h"
#include "edge_se3Switchable.h"
#include "vertex_switchLinear.h"



G2O_REGISTER_TYPE(EDGE_SWITCH_PRIOR, EdgeSwitchPrior);
G2O_REGISTER_TYPE(EDGE_SE3_SWITCHABLE, EdgeSE3Switchable);
G2O_REGISTER_TYPE(VERTEX_SWITCH, VertexSwitchLinear);
