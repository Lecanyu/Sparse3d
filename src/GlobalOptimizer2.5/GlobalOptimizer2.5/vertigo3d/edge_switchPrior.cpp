
#include "edge_switchPrior.h"
using namespace std;


EdgeSwitchPrior::EdgeSwitchPrior() : g2o::BaseMultiEdge<1, double>()
{
	resize(3);
}

bool EdgeSwitchPrior::read(std::istream &is)
{
	double new_measurement;
	is >> new_measurement;

	setMeasurement(new_measurement);

	is >> information()(0, 0);
	return true;
}

bool EdgeSwitchPrior::write(std::ostream &os) const
{
	os << measurement() << " " << information()(0, 0);
	return true;
}

void EdgeSwitchPrior::linearizeOplus()
{
	_jacobianOplus[0].setZero();
	_jacobianOplus[1].setZero();
	_jacobianOplus[2].setZero();

	double sum = 0.0;
	for (int i = 0; i < _vertices.size(); ++i)
	{
		const VertexSwitchLinear* s = static_cast<const VertexSwitchLinear*>(_vertices[i]);
		if (s->x() < 0)
			continue;
		sum += s->x();
	}

	for (int i = 0; i < _vertices.size(); ++i)
	{
		const VertexSwitchLinear* s = static_cast<const VertexSwitchLinear*>(_vertices[i]);
		if (s->x() < 0)
			continue;

		Eigen::Matrix<double, 1, 1> j;
		j << -2 * (measurement() - sum);
		_jacobianOplus[i] = j;
	}
}

void EdgeSwitchPrior::computeError()
{
	double sum = 0.0;
	for (int i = 0; i < _vertices.size(); ++i)
	{
		const VertexSwitchLinear* s = static_cast<const VertexSwitchLinear*>(_vertices[i]);
		if (s->x() < 0)
			continue;
		
		sum += s->x();
	}
	_error[0] = (measurement() - sum)*(measurement() - sum);
}

