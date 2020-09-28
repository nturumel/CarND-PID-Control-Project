#include "Twiddle.h"

void Twiddle::runSpeed()
{
	// create a hub
	uWS::Hub h;
	PID  controllerAngle.Init(_p);
	h.onMessage([&controllerAngle](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length,
		uWS::OpCode opCode)
		{
			if (length && length > 2 && data[0] == '4' && data[1] == '2')
			{
				auto s = hasData(string(data).substr(0, length));
				if (s != "")
				{

				}
			}
		}


	// create a PID

}