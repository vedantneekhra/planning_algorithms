#include<iostream>
#include<vector>
#include<unordered_map>
#include<fstream>
#include<unordered_set>

#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#define w 1000

using namespace std;

class node{
public:
  int x , y;
  node(int x1 , int y1){
    x = x1;
    y = y1;
  }
  float local;
  node* parent;
};

unordered_set<string> obs;            //obstacle map
// unordered_map<string , node* > map;
ifstream path("path.txt");            //Path File

#include <string>
#include <fstream>
#include <unistd.h>


string its(int x){                   //int to string converter
  std::ostringstream s_;
  s_ << x;
	return s_;
}

vector<string> split(const string& s, char delimiter)
{  //string splitting
   vector<string> tokens;
   string token;
   istringstream tokenStream(s);
   while (getline(tokenStream, token, delimiter))
   {
      tokens.push_back(token);
   }
   return tokens;
}

node node_without_obs(node& start){
   //if any obstacle is detected on the path then it moves on the path skips the
   //path cover by obstacle and return the first pixel on which obstacle is not present
  string line;
  int x,y;
  if(path.is_open()){
    while(getline(path,line)){
      if(obs.find(line) == obs.end()){
          vector<string> coor = split(line , ',');
          stringstream x_coor(coor[0]);
          stringstream y_coor(coor[1]);
          x_coor >> x;
          y_coor >> y;
          break;
      }
    }
  }
  node clear_node(x,y);
  return clear_node;
}

void move_around_obs(node& start, node& end){
  //bug algorithm
  //moves around the complete obstacle
  //intially we move in anticlockwise direction around the complete obstacle to find which direction to move to reach goal Position
  //so that shortest path is selected.
  vector<pair<int,int>> dir = {{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1},{-1,0},{-1,1}};       //close neighbor
  start.local = 0;                                                                          //distance from start position or end position
  node *crt_node = &start;
  while(!((end.x == crt_node->x)&(end.y == crt_node->y))){                                 //moving from start position to goal position
    // cout<<crt_node->x<<' '<<crt_node->y<<'\n';
    for(int i = 0; i<dir.size(); i++){
      if(obs.find(its(crt_node->x + dir[(i>0)?(i-1):(dir.size()-1)].first)+','+its((crt_node->y + dir[(i>0)?(i-1):(dir.size()-1)].second)))==obs.end()){
        continue;                                                                         //to follow the boundaries of the object
      }
      int new_x = crt_node->x + dir[i].first;                                             //
      int new_y = crt_node->y + dir[i].second;                                            //
      string tocheck = its(new_x)+','+its(new_y);
      if(obs.find(tocheck)==obs.end()){                                                   //we check in all direction for obstacle but mmove only along x or y so that
        i+=i%2;                                                                           //obstacle is completely covered
        if(i==dir.size()){
          i = 0;
        }
        new_x = crt_node->x + dir[i].first;
        new_y = crt_node->y + dir[i].second;
        node *n = new node(new_x,new_y);
        n->local = crt_node->local + sqrt(dir[i].first*dir[i].first+dir[i].second*dir[i].second);
        crt_node->parent = n;
        crt_node = n;
        break;
      }
    }
  }
  end = *crt_node;
  int  j = 0;
  while(!((start.x == crt_node->x)&(start.y == crt_node->y))){                          //moving from goal position to start position
    cout<<crt_node->x<<' '<<crt_node->y<<'\n';
    for(int i = 0; i<dir.size(); i++){
      if(obs.find(its(crt_node->x + dir[(i>0)?(i-1):(dir.size()-1)].first)+','+its((crt_node->y + dir[(i>0)?(i-1):(dir.size()-1)].second)))==obs.end()){
        continue;
      }
      int new_x = crt_node->x + dir[i].first;
      int new_y = crt_node->y + dir[i].second;
      string tocheck = its(new_x)+','+its(new_y);
      if(obs.find(tocheck)==obs.end()){
        i+=i%2;
        if(i==dir.size()){
          i = 0;
        }
        new_x = crt_node->x + dir[i].first;
        new_y = crt_node->y + dir[i].second;
        node *n = new node(new_x,new_y);
        n->local = (j==0)?0:(crt_node->local);
        n->local += sqrt(dir[i].first*dir[i].first+dir[i].second*dir[i].second);
        n->parent = crt_node;
        crt_node = n;
        j=1;
        break;
      }
    }
  }
  if(end.local > crt_node->local){                                                      //which direction to move
    start = *crt_node;
  }
}

int main(){


  char win[] = "Dynamic Path Planning";
  cv::Mat pathimg = cv::Mat::zeros( w, w, CV_8UC3 );

  //////////////////////////////obstacle///////////////////////////////////////////////
  int cen_y = 100;
  int cen_x = 67;
  int radius = 25;
  int y = cen_y - radius;
  int x = cen_x;
  while (y <= (cen_y + radius)){
    int chdlnth = (int)sqrt(radius*radius - (y - cen_y)*(y - cen_y));
    x = cen_x-chdlnth;
    while( x <= (cen_x+chdlnth)){
      string tocheck = (its(x)+','+its(y));
      obs.insert(tocheck);
      // cout<<tocheck<<'\n';
      // circle( pathimg, cv::Point(w/2+x , w/1.5 - y), 1, cv::Scalar( 0, 0, 255 ), cv::FILLED, cv::LINE_8 );
      x++;
    }
    y++;
  }
  int inf_radius = 15;
  y = cen_y - inf_radius;
  x = cen_x;
  while (y <= (cen_y + inf_radius)){
    int chdlnth = (int)sqrt(inf_radius*inf_radius - (y - cen_y)*(y - cen_y));
    x = cen_x-chdlnth;
    while( x <= (cen_x+chdlnth)){
      string tocheck = (its(x)+','+its(y));
      // obs.insert(tocheck);
      // cout<<tocheck<<'\n';
      circle( pathimg, cv::Point(w/2+x , w/1.5 - y), 1, cv::Scalar( 0, 0, 255 ), cv::FILLED, cv::LINE_8 );
      x++;
    }
    y++;
  }
  /////////////////////////////////////////////////////////////////////////////////

  int coll = 0;
  string line;
  // int x,y;
  if(path.is_open()){
    while(getline (path,line)){                                                        //program runs till path file in completely read.
      cout<<line<<'\n';
      if(obs.find(line) == obs.end()){                                                 //moves on the path given till no obstacle is detected
        vector<string> coor = split(line , ',');
        stringstream x_coor(coor[0]);
        stringstream y_coor(coor[1]);
        x_coor >> x;
        y_coor >> y;
        circle( pathimg, cv::Point(w/2+x , w/1.5 - y), 1, cv::Scalar( 255, 0, 0 ), cv::FILLED, cv::LINE_8 );
      }
      else{                                                                            //if obstacle is detected
        node start(x,y);
        node end = node_without_obs(start);
        circle( pathimg, cv::Point(w/2+end.x , w/1.5 - end.y), 1, cv::Scalar( 255, 0, 0 ), cv::FILLED, cv::LINE_8 );
        cout<<"start "<<start.x<<' '<<start.y<<'\n';
        cout<<"end "<<end.x<<' '<<end.y<<'\n';
        move_around_obs(start, end);
        node *n = &start;
        while(!((n->x==end.x)&(n->y==end.y))){
          cout<<n->x<<' '<<n->y<<'\n';
          circle( pathimg, cv::Point(w/2+n->x , w/1.5 - n->y), 1, cv::Scalar( 255, 0, 0 ), cv::FILLED, cv::LINE_8 );
          n=n->parent;
        }
      }
    }
    imshow(win, pathimg);
    cv::waitKey(0);
    path.close();
  }

}
