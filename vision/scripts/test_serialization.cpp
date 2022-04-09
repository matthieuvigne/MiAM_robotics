#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <common/marker.hpp>
#include <common/time.hpp>

int main(int argc, char* argv[])
{
  //~ // Solution 1
  //~ unsigned char ubytes[] = {72,101,108,108,111,' ',119,111,114,108,100,'\0'};
  //~ int n = sizeof(ubytes);
  //~ char uchars[n+1];
  //~ memcpy(uchars, ubytes, sizeof(ubytes));
  //~ uchars[n] = '\0';
  //~ std::cout << sizeof(ubytes) << std::endl;
  //~ std::cout << sizeof(uchars) << std::endl;
  //~ std::cout << uchars << std::endl;
  //~ std::cout << static_cast<unsigned int>(ubytes[5]) << std::endl;

  //~ // Solution 2
  //~ // String occupies at least 32 bytes (capacity) but it does not mean it is filled with it.
  //~ char bytes[] = {72,101,108,108,111,' ',119,111,114,108,100,'\0'};
  //~ std::string chars(bytes, sizeof(bytes));
  //~ std::cout << sizeof(bytes) << std::endl;
  //~ std::cout << sizeof(chars) << std::endl;
  //~ std::cout << chars.size() << std::endl;
  //~ std::cout << chars << std::endl;

  //~ // Get the bytes from the string
  //~ std::vector<char> cbytes(chars.cbegin(), chars.cend());
  //~ for(char const& c : cbytes)
    //~ std::cout << c << std::endl;

  //~ double x = 125;
  //~ int n = sizeof(x);
  //~ std::cout << sizeof(x) << std::endl;
  //~ char* ptr = (char*)(&x);
  //~ std::vector<unsigned char> chars(&x,&x+sizeof(x));
  //~ double a = *((double*) ptr);
  //~ std::cout << a << std::endl;

  // Solution 3
  
  //~ // Build a custom marker estimate
  //~ common::Marker sent_marker;
  //~ sent_marker.id = 42;
  //~ sent_marker.family = common::MarkerFamily::CENTRAL_MARKER;
  //~ sent_marker.timestamp_ns = 1000u;
  //~ sent_marker.T_WM = Eigen::Affine3d::Identity();
  //~ sent_marker.cov_T_WM = Eigen::Matrix<double,6,6>::Identity();
  //~ std::cout << sent_marker.print() << std::endl;
  //~ std::vector<char> sent_message;
  //~ sent_marker.serialize(&sent_message);
  //~ std::string const sent_message_str(sent_message.cbegin(), sent_message.cend());
  
  //~ // Deserialize the marker estimate
  //~ std::vector<char> received_message(sent_message_str.cbegin(), sent_message_str.cend());
  //~ common::Marker received_marker;
  //~ received_marker.deserialize(received_message);
  //~ std::cout << received_marker.print() << std::endl;

  //~ std::vector<char> chars{'h','e','l','l','o','\0','w','o','r','l','d','\0'};
  //~ std::string str(chars.data(), chars.size());
  //~ std::cout << str << std::endl;
  //~ std::size_t pos1 = str.find('\0');
  //~ std::cout << pos1 << std::endl;
  //~ std::size_t pos2 = str.find('\0',pos1+1);
  //~ std::cout << pos2 << std::endl;
  //~ std::size_t n = std::count(str.cbegin(), str.cend(), '\0');
  //~ std::cout << n << std::endl;

  std::vector<int> v(10);
  size_t const distance = std::distance(v.cbegin(), v.cend());
  std::cout << "Distance is " << distance << std::endl;
  //~ std::vector<int>::const_iterator it = v.cbegin();
  //~ std::advance(it,11);
  //~ if(it == v.cend())
    //~ std::cout << "ok" << std::endl;
  //~ else
    //~ std::cout << "nok" << std::endl;

  //~ std::string::iterator it = std::find("hello",'l');

  return EXIT_SUCCESS;
}

