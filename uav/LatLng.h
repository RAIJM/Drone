class LatLng{

  private:
    float lat;
    float lng;
  
  public:
    LatLng()
    {
        
    }

    LatLng(float lat, float lng)
	{
      this->lat = lat;
	  this->lng = lng;
	}

    float get_lat()
    {
      return this->lat;
    }

    float get_lng()
    {
      return this->lng;
    }

    void set_lat(float lat)
    {
        this->lat = lat;
    }

    void set_lng(float lon)
    {
      this->lng = lon;  
    }

	
};
