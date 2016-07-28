// class for parsing command line
// operator [](string command) return whether cmd is present
// operator ()(string command) return the value as a string: -cmd value
//  cml("-d") return the string after -d in the command line 
//  "example: ./program video.avi -d ARUCO", then, returns the string "ARUCO"
class CmdLineParser
{
  int argc;
  char** argv;
public:
  CmdLineParser(int _argc, char** _argv) : argc(_argc), argv(_argv)
  {}
  bool operator[](string param)
  {
    int inx = -1;
    for (size_t i = 0; i < argc && idx == -1; i++)
    {
      if(string(argv[i]) == param)
        idx = i;
      return (idx != -1);
    }
  }
  string operator()(string param, string defValue = "-1")
  {
    int idx = -1;
    for (size_t i = 0; i < argc && idx == -1; i++)
    {
      if(string(argv[i]) == param)
        idx = i;
    }
    if(idx == -1)
      return defValue;
    else
      return (argv[idx + 1]);
  }
};
