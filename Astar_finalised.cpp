#include<iostream>
#include<algorithm>
#include<unordered_map>
#include<string>
#include<vector>
#include<math.h>
#include<unordered_set>

#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

#define w 1000

class node{
  public:
    int x, y;
    bool visited = false;
    node* parent;
    float g = 0;
    float heroistic = 0;
    float func = 0;
    bool obs = 0;

    node(int x1 , int y1){
      x = x1;
      y = y1;
    }

    void find_heroistic(node& end){
      heroistic = sqrt((end.x - x)*(end.x - x) + (end.y - y)*(end.y - y));
    }
};

unordered_map<string , node*> pmap;

bool operator < (const node& lhs, const node& rhs)
{
    return lhs.func < rhs.func;
}

bool operator > (const node& lhs, const node& rhs)
{
    return lhs.func > rhs.func;
}

string its(int x){
	string s;
  bool neg = false;
  if(x<0){
    neg = true;
  }
  x = abs(x);
	if(x!=0){
		for(int i = 1; x != 0; i++){
			int j = x%10;
			j += 48;
			x = x/10;
			s.insert(0,1,(char)j);
			//cout<<j;
		}
	}
	else{
		s = '0';
	}
  if(neg){
    s.insert(0,1,'-');
  }
	return s;
}

bool compareInterval(node* i1, node* i2)
{
  if(i1->func != i2->func){
    return (i1->func < i2->func);
  }
  else{
    return (i1->heroistic < i2->heroistic);
  }
}


vector<node*> Astar(node& start, node& end, int forward_radius, node& search_from){
  vector<node*>  q;
  vector<pair<int,int> > dir = {{0,1},{0,-1},{1,0},{-1,0},{1,1},{1,-1},{-1,1},{-1,-1}};
  vector<node*> r;
  unordered_map<string,node*> map{pmap};
  unordered_set<string> q_check;
  forward_radius -= 1;
  int path_posi = 0;
  for(int i = 0; i < dir.size(); i++){
    node* crtnode = &search_from;
    int new_x = crtnode->x+dir[i].first;
    int new_y = crtnode->y+dir[i].second;
    string tocheck = (its(new_x)+','+its(new_y));
    if(map.find(tocheck) != map.end()){
      if(map[tocheck]->obs){
        path_posi = 0;
        map[its(start.x)+','+its(start.y)] = &start;
        q.push_back(&start);
        break;
      }
    }
    if(i == (dir.size()-1)){
      path_posi = 1;
      map[its(search_from.x)+','+its(search_from.y)] = &search_from;
      q.push_back(&search_from);
    }
  }

  start.visited = true;
  //start.visited = 0;
  // cout<< its(start.x)+','+its(start.y);//(*m[its(start.x)+','+its(start.y)]).visited<<'\n';
  int j = 0;
  bool limit = 0;
  while((!end.visited)&(!limit)&(q.size())){
    // cout<<"q.size() :"<<q.size()<<'\n';
    // showpq(q);
    sort(q.begin() , q.end(), compareInterval);
    node *crtnode = q[0];
    // for(int i = 0; i < q.size();i++){
    //   cout<<(*q[i]).func<<" ";
    // }
    // cout<<"episode start "<<crtnode->x<<" "<<crtnode->y<<'\n';
    q.erase(q.begin());
    if(sqrt((start.x-crtnode->x)*(start.x-crtnode->x) + (start.y-crtnode->y)*(start.y-crtnode->y)) > forward_radius){
      limit = 1;
      continue;
    }

    // q_check.erase(its(crtnode->x)+','+its(crtnode->y));

    // j++;
    for(int i = 0; i < dir.size(); i++){
      // if(!check_obs(crtnode.x+dir[i].first , crtnode.y + dir[i].second){
        int new_x = crtnode->x+dir[i].first;
        int new_y = crtnode->y+dir[i].second;
        if(((crtnode->x+dir[i].first) == end.x)&(crtnode->y +dir[i].second == end.y)){
          end.g= crtnode->g+sqrt(dir[i].first*dir[i].first + dir[i].second*dir[i].second);
          end.func = end.g;
          end.visited = true;
          end.parent = crtnode;
          q.push_back(&end);
          //cout<<"here";
          break;
        }
        string tocheck = (its(new_x)+','+its(new_y));

        // cout<<tocheck<<'\n';

        if(map.find(tocheck) == map.end()){
          node* nnode = new node (new_x,new_y);
          // cout<<(*nnode).x<<" "<<(*nnode).y<<'\n';
          // cout<<(int)crtnode.x<<' '<<dir[i].first<<" "<<crtnode.y + dir[i].second<<'\n';
          (*nnode).g = crtnode->g + sqrt(dir[i].first*dir[i].first + dir[i].second*dir[i].second);
          (*nnode).find_heroistic(end);
          (*nnode).func = (*nnode).g + (*nnode).heroistic;
          // cout<<"g "<<( *nnode).g<<"h "<<(*nnode).heroistic<<'\n';
          (*nnode).parent = crtnode;
          (*nnode).visited = true;
          q.push_back(nnode);
          q_check.insert(tocheck);
          map[tocheck] = nnode;
          // cout<<
        }
        else{
          if(!map[tocheck]->obs){
              int func = sqrt((end.x-new_x)*(end.x-new_x) + (end.y-new_y)*(end.y-new_y)) + crtnode->g + sqrt(dir[i].first*dir[i].first + dir[i].second*dir[i].second);
              node *cnode = map[tocheck];
              if(func < cnode->func){
                cnode->g = crtnode->g + sqrt(dir[i].first*dir[i].first + dir[i].second*dir[i].second);
                cnode->func = func;
                cnode->parent = crtnode;
              }
              if(func <= cnode->func){
                if(q_check.find(tocheck) == q_check.end()){
                  q.push_back(cnode);
                  q_check.insert(tocheck);
                }
              }
            }
          }
      // }
    }
  }
  sort(q.begin() , q.end(), compareInterval);
  node* n = q[0];
  // cout<<n<<'\n';
  // cout<<"path : ";
  // cout<<n->x<<' '<<n->y<<'\n';
  // cout<<n->parent->x<<' '<<n->parent->y<<' ';
  while(!((start.x == n->parent->x)&(start.y == n->parent->y))){
    cout<<n->x<<' '<<n->y<<'\n';
    n = n->parent;
  }
  r.push_back(n);
  r.push_back(q[0]);
  return  r;
}

int main(){
  char win[] = "Dynamic Path Planning";
  cv::Mat pathimg = cv::Mat::zeros( w, w, CV_8UC3 );
  int cen_y = 250;
  int cen_x = 0;
  int radius = 150;
  int y = cen_y - radius;
  int x = cen_x;
  while (y <= (cen_y + radius)){
    int chdlnth = (int)sqrt(radius*radius - (y - cen_y)*(y - cen_y));
    x = cen_x-chdlnth;
    while( x <= (cen_x+chdlnth)){
      node* temp = new node(x , y);
      temp->obs = 1;
      string tocheck = (its(x)+','+its(y));
      pmap[tocheck] = temp;
      // cout<<tocheck<<'\n';
      x++;
      circle( pathimg, cv::Point(w/2+x , w/1.5 - y), 1, cv::Scalar( 0, 0, 255 ), cv::FILLED, cv::LINE_8 );
    }
    y++;
  }
  // pmap = map;
  // cout<<&map<<" "<<&pmap<<'\n';
  imshow( win, pathimg );
  // moveWindow( , 0, 200 );
  node start(0,0) , end(0,475);
  start.find_heroistic(end);
  start.func = start.heroistic;
  vector<node*> path = Astar(start, end, 70 , start);
  // cout<<end.parent->parent->y<<'\n';

  cout<<path[0]->x<<" "<<path[0]->y<<'\n';
  while(!((path[0]->x == end.x)&(path[0]->y == end.y))){
    path = Astar(*path[0], end, 70 , *path[1]);
    cout<<"here \n";
    cout<<"to_move "<<path[0]->x<<" "<<path[0]->y<<'\n';
    cout<<"extrema "<<path[1]->x<<" "<<path[1]->y<<'\n';
    circle( pathimg, cv::Point(w/2+path[0]->x , w/1.5 - path[0]->y), 5, cv::Scalar( 255, 0, 0 ), cv::FILLED, cv::LINE_8 );
    imshow(win, pathimg);
    // cv::wait(0.5);
  }
  cv::waitKey( 0);
  //cout<<its(10);


}
