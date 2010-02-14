#include "HSHumanoidNodeParser.hpp"

namespace dynamicsJRLJapan
{
  namespace HumanoidSpecificitiesData {
    
    
    namespace fusion = boost::fusion;
    namespace phoenix = boost::phoenix;
    namespace qi = boost::spirit::qi;
    namespace ascii = boost::spirit::ascii;
    
    int ReadXMLData3(std::string &aFileName,
		     HumanoidNode &ast)
    {
      std::ifstream in((char *)aFileName.c_str(), std::ios_base::in);
      
      if (!in)
	{
	  std::cerr << "Error: Could not open input file: "
		    << aFileName << std::endl;
	  return 1;
	}
      
      std::string storage; // We will read the contents here.
      in.unsetf(std::ios::skipws); // No white space skipping!
      std::copy(
		std::istream_iterator<char>(in),
		std::istream_iterator<char>(),
		std::back_inserter(storage));
      
      struct HumanoidNode_parser<std::string::const_iterator> 
	hsxml; // Our grammar

      
      using boost::spirit::ascii::space;
      std::string::const_iterator iter = storage.begin();
      std::string::const_iterator end = storage.end();
      
      // this will print something like: boost::fusion::vector2<int, double>
      display_attribute_of_parser(hsxml);
      
      bool r = phrase_parse(iter, end, hsxml, space, ast);
      
      if (r && iter == end)
	{
	  std::cout << "-------------------------\n";
	  std::cout << "Parsing succeeded\n";
	  std::cout << "-------------------------\n";
	  return 0;
	}
      else
	{
	  std::string::const_iterator some = iter+30;
	  std::string context(iter, (some>end)?end:some);
	  std::cout << "-------------------------\n";
	  std::cout << "Parsing failed\n";
	  std::cout << "stopped at: \": " << context << "...\"\n";
	  std::cout << "-------------------------\n";
	  return 1;
	}
    }
  };  
};

int main()
{
  namespace dhs=dynamicsJRLJapan::HumanoidSpecificitiesData;
  std::string aFileName("/home/stasse/devel/openrobots/share/hrp2_14/HRP2Specificities.xml");
  dhs::HumanoidNode ahn;
  dhs::ReadXMLData3(aFileName,ahn);
  std::cout <<ahn <<std::endl;
}
