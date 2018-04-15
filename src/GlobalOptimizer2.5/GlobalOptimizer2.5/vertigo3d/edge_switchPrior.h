#pragma once


#include "vertex_switchLinear.h"
#include <g2o/core/base_multi_edge.h>


class EdgeSwitchPrior : public g2o::BaseMultiEdge<1, double>
{
  public:
    EdgeSwitchPrior();

	virtual bool read(std::istream& is);
	virtual bool write(std::ostream& os) const;
    void linearizeOplus();
    void computeError();

	virtual void setMeasurement(const double m){
		_measurement = m;
	}


};
