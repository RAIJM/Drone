


float toPWM(float value)
{
	float iMin = -50;
	float iMax = 50;
	return (int) ((value - iMin) * (1000) / (iMax - iMin) + 1000);
}

float radians_to_degrees(float x)
{
    return x / 180.0 * 3.14;
}


