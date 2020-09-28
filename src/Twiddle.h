#pragma once
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"
#include <climits>

class Twiddle
{
	vector<double> _p{3, 1};
	vector<double> _dp{3, 0.1};
	double _tolerance = 0.02;
	int _iter = 100;
	double _bestError = DBL_MAX;
	void runSteering();
	void runSpeed();

public:
	vector<double> optimiseSteering();
	vector<double> optimiseSpeed();

};