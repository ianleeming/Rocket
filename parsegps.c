

typedef struct gpgga_s {
  
  int16_t utc_hr;
  int16_t utc_min;
  int16_t utc_sec;
  int16_t utc_ms;

  int16_t lat_deg; // - for south
  int16_t lat_min;
  int16_t lat_min_10000;

  int16_t lon_deg; // - for east
  int16_t lon_min;
  int16_t lon_min_10000;

  int8_t fix;
  int8_t nsat;
  
} gpgga_t;



void parse_gps(char *p)
{
  if (p[0] == '$') {
    if (p[1] == 'G' && p[2] == 'P' && p[3] == 'G' && p[4] == 'G' && p[5] == 'A') {
      parse_gpgga(p+6);
      return;
    }
  }
}

void parse_gpgga(char *p)
{
  while (*p == ',' || *p == ' ') p++;
  
  
}
