#include "ros/ros.h"
#include <gnss_comm/gnss_constant.hpp>
#include "gnss_comm/gnss_ros.hpp"
#include "gnss_comm/gnss_utility.hpp"
#include "rtklib.h"

constexpr char opt_file[] = "rtkrcv.conf";
constexpr int MAXSTR = 1024;      

static char passwd[MAXSTR] = "admin";     
static int timetype = 0;             
static int soltype = 0;             
static int solflag = 2;            
static int strtype[] = {                  
   STR_SERIAL, STR_NONE, STR_NONE, STR_NONE, STR_NONE, STR_NONE, STR_NONE, STR_NONE
};
static char strpath[8][MAXSTR] = {"", "", "", "", "", "", "", ""}; 
static int strfmt[] = {   
   STRFMT_UBX, STRFMT_RTCM3, STRFMT_SP3, SOLF_LLH, SOLF_NMEA
};
static int nmeareq = 0; 
static int svrcycle = 10;  
static int timeout = 10000;     
static int reconnect = 10000;
static int nmeacycle = 5000;
static int buffsize = 32768; 
static int navmsgsel = 0; 
static char proxyaddr[256] = "";  
static int fswapmargin = 30; 
static char startcmd[MAXSTR] = ""; 
static char stopcmd [MAXSTR] = ""; 
static double nmeapos[] = { 0, 0, 0 }; 
static char rcvcmds[3][MAXSTR] = { "" }; 

#define TIMOPT  "0:gpst,1:utc,2:jst,3:tow"
#define CONOPT  "0:dms,1:deg,2:xyz,3:enu,4:pyl"
#define FLGOPT  "0:off,1:std+2:age/ratio/ns"
#define ISTOPT  "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,6:ntripcli,7:ftp,8:http"
#define OSTOPT  "0:off,1:serial,2:file,3:tcpsvr,4:tcpcli,6:ntripsvr,11:ntripc_c"
#define FMTOPT  "0:rtcm2,1:rtcm3,2:oem4,3:oem3,4:ubx,5:ss2,6:hemis,7:skytraq,8:gw10,9:javad,10:nvs,11:binex,12:rt17,13:sbf,14:cmr,15:tersus,18:sp3"
#define NMEOPT  "0:off,1:latlon,2:single"
#define SOLOPT  "0:llh,1:xyz,2:enu,3:nmea,4:stat"
#define MSGOPT  "0:all,1:rover,2:base,3:corr"

static opt_t rcvopts[]={
    {"console-passwd",  2,  (void *)passwd,              ""     },
    {"console-timetype",3,  (void *)&timetype,           TIMOPT },
    {"console-soltype", 3,  (void *)&soltype,            CONOPT },
    {"console-solflag", 0,  (void *)&solflag,            FLGOPT },
    
    {"inpstr1-type",    3,  (void *)&strtype[0],         ISTOPT },
    {"inpstr2-type",    3,  (void *)&strtype[1],         ISTOPT },
    {"inpstr3-type",    3,  (void *)&strtype[2],         ISTOPT },
    {"inpstr1-path",    2,  (void *)strpath [0],         ""     },
    {"inpstr2-path",    2,  (void *)strpath [1],         ""     },
    {"inpstr3-path",    2,  (void *)strpath [2],         ""     },
    {"inpstr1-format",  3,  (void *)&strfmt [0],         FMTOPT },
    {"inpstr2-format",  3,  (void *)&strfmt [1],         FMTOPT },
    {"inpstr3-format",  3,  (void *)&strfmt [2],         FMTOPT },
    {"inpstr2-nmeareq", 3,  (void *)&nmeareq,            NMEOPT },
    {"inpstr2-nmealat", 1,  (void *)&nmeapos[0],         "deg"  },
    {"inpstr2-nmealon", 1,  (void *)&nmeapos[1],         "deg"  },
    {"inpstr2-nmeahgt", 1,  (void *)&nmeapos[2],         "m"    },
    {"outstr1-type",    3,  (void *)&strtype[3],         OSTOPT },
    {"outstr2-type",    3,  (void *)&strtype[4],         OSTOPT },
    {"outstr1-path",    2,  (void *)strpath [3],         ""     },
    {"outstr2-path",    2,  (void *)strpath [4],         ""     },
    {"outstr1-format",  3,  (void *)&strfmt [3],         SOLOPT },
    {"outstr2-format",  3,  (void *)&strfmt [4],         SOLOPT },
    {"logstr1-type",    3,  (void *)&strtype[5],         OSTOPT },
    {"logstr2-type",    3,  (void *)&strtype[6],         OSTOPT },
    {"logstr3-type",    3,  (void *)&strtype[7],         OSTOPT },
    {"logstr1-path",    2,  (void *)strpath [5],         ""     },
    {"logstr2-path",    2,  (void *)strpath [6],         ""     },
    {"logstr3-path",    2,  (void *)strpath [7],         ""     },
    
    {"misc-svrcycle",   0,  (void *)&svrcycle,           "ms"   },
    {"misc-timeout",    0,  (void *)&timeout,            "ms"   },
    {"misc-reconnect",  0,  (void *)&reconnect,          "ms"   },
    {"misc-nmeacycle",  0,  (void *)&nmeacycle,          "ms"   },
    {"misc-buffsize",   0,  (void *)&buffsize,           "bytes"},
    {"misc-navmsgsel",  3,  (void *)&navmsgsel,          MSGOPT },
    {"misc-proxyaddr",  2,  (void *)proxyaddr,           ""     },
    {"misc-fswapmargin",0,  (void *)&fswapmargin,        "s"    },
    
    {"misc-startcmd",   2,  (void *)startcmd,            ""     },
    {"misc-stopcmd",    2,  (void *)stopcmd,             ""     },
    
    {"file-cmdfile1",   2,  (void *)rcvcmds[0],          ""     },
    {"file-cmdfile2",   2,  (void *)rcvcmds[1],          ""     },
    {"file-cmdfile3",   2,  (void *)rcvcmds[2],          ""     },
    
    {"",0,NULL,""}
};

static rtksvr_t svr;  
static prcopt_t prcopt;  
static solopt_t solopt[2] = { {0} }; 
static filopt_t filopt = { "" };  

static ros::Publisher pos_pub;

extern "C" void publish_position(const sol_t *sol)
{
   gnss_comm::GnssPVTSolnMsg pvt_msg;

   int gps_week;
   pvt_msg.time.tow = time2gpst(sol->time, &gps_week);
   pvt_msg.time.week = gps_week;

   switch (sol->stat) {
   case SOLQ_SINGLE:
      pvt_msg.fix_type = 3;
      break;
   case SOLQ_PPP:   
      pvt_msg.fix_type = 6;
      break;
   case SOLQ_NONE:
   default: 
      pvt_msg.fix_type = 0;
      break;   
   }

   pvt_msg.valid_fix = pvt_msg.fix_type != 0 ? 1 : 0;
   pvt_msg.diff_soln = 0;
   pvt_msg.carr_soln = 0;
   pvt_msg.num_sv = sol->ns;

   double pos[3];
   ecef2pos(sol->rr, pos);

   pvt_msg.latitude = pos[0]*R2D;
   pvt_msg.longitude = pos[1]*R2D;
   pvt_msg.altitude = pos[2];
   pvt_msg.height_msl = pos[2] - geoidh(pos);

   pvt_msg.h_acc = 0;
   pvt_msg.v_acc = 0;
   pvt_msg.p_dop = 0;

   double vel[3];
   ecef2enu(pos, sol->rr+3, vel);

   pvt_msg.vel_n = vel[1];
   pvt_msg.vel_e = vel[0];
   pvt_msg.vel_d = -vel[2];
   pvt_msg.vel_acc = 0;

   pos_pub.publish(pvt_msg);
}

static int start_rtklib()
{
   double npos[3] = { 0, 0, 0 };
   char *ropts[] = { "", "", "" };
   char *paths[] = {
      strpath[0], strpath[1], strpath[2], strpath[3], strpath[4], strpath[5],
      strpath[6], strpath[7]
   };
   char s1[3][MAXRCVCMD] = { "", "", "" }, *cmds[] = { NULL, NULL, NULL };
   char s2[3][MAXRCVCMD] = { "", "", ""}, *cmds_periodic[] = { NULL, NULL, NULL };
   char errmsg[2048] = "";
   int stropt[8] = { 0 };
    
   stropt[0] = timeout;
   stropt[1] = reconnect;
   stropt[2] = 1000;
   stropt[3] = buffsize;
   stropt[4] = fswapmargin;
   strsetopt(stropt);
    
   solopt[0].posf=strfmt[3];
   solopt[1].posf=strfmt[4];
    
   if (!rtksvrstart(&svr, svrcycle, buffsize, strtype, paths, strfmt, navmsgsel,
                  cmds, cmds_periodic, ropts, nmeacycle, nmeareq, npos, &prcopt,
                  solopt, nullptr, errmsg)) {
      ROS_ERROR("rtklib start error (%s)\n", errmsg);
      return 0;
   }

   return 1;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "rtkrcv");

   ros::NodeHandle node;

   pos_pub = node.advertise<gnss_comm::GnssPVTSolnMsg>("pvt", 100);

   std::string config_file, output_dir;
   node.getParam("rtkrcv/config_file", config_file);
   node.getParam("rtkrcv/output_dir", output_dir);

   ROS_INFO("config file: %s", config_file.c_str());
   
   rtksvrinit(&svr);

   resetsysopts();
   if (!loadopts(config_file.c_str(), rcvopts) || !loadopts(config_file.c_str(), sysopts)) {
      ROS_ERROR("no options file: %s.", opt_file);
      exit(-1);
   }
   getsysopts(&prcopt, solopt, &filopt);

   std::string output_file = output_dir + "/" + strpath[4];
   strcpy(strpath[4], output_file.c_str());
   ROS_INFO("output path: %s", strpath[4]);

   if (!start_rtklib())
      return -1;

   ros::spin();

   return 0;
}
