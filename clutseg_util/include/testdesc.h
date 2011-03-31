/*
 * Author: Julius Adorf
 */

#include <string>
#include <map>
#include <set>

using namespace std;

typedef map<string, set<string> > TestDesc;

TestDesc loadTestDesc(const string & filename);

