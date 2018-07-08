


float toPWM(float value)
{
	float iMin = -50;
	float iMax = 50;
	return (int) ((value - iMin) * (1000) / (iMax - iMin) + 1000);
}

float radians_to_degrees(float x)
{
    return x * 57.2958;
}


float degrees_to_radians(float x)
{
	return x * 0.0174533;
}


class low_pass
{
    private:
        float filter;
        float filter_past;
        float dt;
        float filter_bandwidth;

   public:
        low_pass(float bandwidth, float dt)
        {
		    this->filter_bandwidth = bandwidth;
		    this->dt = dt;
		    this->filter = filter;
		    this->filter_past = 0.0;
	    }

		float update(float current_value)
		{
			this->filter = this->filter_past + this->dt * (this->filter_bandwidth * (current_value - this->filter_past));
		    this->filter_past = this->filter;
		    return this->filter;
		}
};

struct LatLng{
    float lat;
    float lng;

};


