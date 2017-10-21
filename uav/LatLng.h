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

    get_lat()
    {
      return this->lat;
    }

    get_lng()
    {
      return this->lng;
    }

	
};
