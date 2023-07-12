__attribute__((used,section(".pad"))) static const struct {
  char build[6];
  char iso_build_date[10];
  char iso_build_time[10];
  char version[];
} buildstring = {
  "Build ",
  {
     // YYYY-
     __DATE__[7], __DATE__[8], __DATE__[9], __DATE__[10], '-',

     // First month letter, Oct Nov Dec = '1' otherwise '0'
     (__DATE__[0] == 'O' || __DATE__[0] == 'N' || __DATE__[0] == 'D') ? '1' : '0',

     // Second month letter
     (__DATE__[0] == 'J') ? ( (__DATE__[1] == 'a') ? '1' :       // Jan, Jun or Jul
                              ((__DATE__[2] == 'n') ? '6' : '7') ) :
     (__DATE__[0] == 'F') ? '2' :                                // Feb
     (__DATE__[0] == 'M') ? (__DATE__[2] == 'r') ? '3' : '5' :   // Mar or May
     (__DATE__[0] == 'A') ? (__DATE__[1] == 'p') ? '4' : '8' :   // Apr or Aug
     (__DATE__[0] == 'S') ? '9' :                                // Sep
     (__DATE__[0] == 'O') ? '0' :                                // Oct
     (__DATE__[0] == 'N') ? '1' :                                // Nov
     '2',                                                        // Dec
     '-',

     // First day letter, replace space with digit
     (__DATE__[4]==' ') ? '0' : __DATE__[4],

     // Second day letter
     __DATE__[5]
  },
  "T" __TIME__ " ",
  "Version 0.0.0 "
};
