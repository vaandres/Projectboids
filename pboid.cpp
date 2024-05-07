#include <SFML/Graphics.hpp>
#include <cmath>  //libreria usata solitamente?
#include <iostream>
#include <random>
#include <vector>

class boid {
 private:
  std::vector<double> position_{0, 0};
  std::vector<double> velocity_{0, 0};
  // Costruttore della classe boids
 public:
  boid(double x, double y, double vx, double vy)
      : position_{x, y}, velocity_{vx, vy} {}  // spazi leggibilità 

  std::vector<double> position() const { return position_; };

  std::vector<double> velocity() const { return velocity_; };
  void setPosition(const std::vector<double>& newPos) { position_ = newPos; }
  void setVelocity(const std::vector<double>& newVel) { velocity_ = newVel; }
};
double dist(boid const& b1, boid const& b2) {
  auto pos1 = b1.position();
  auto pos2 = b2.position();
  return std::sqrt(std::pow(pos1[0] - pos2[0], 2) +     //conservare velocità totale
                   std::pow(pos1[1] - pos2[1], 2));
}
std::vector<double> operator+(std::vector<double> v1, std::vector<double> v2) {
  auto vxf = v1[0] + v2[0];
  auto vyf = v1[1] + v2[1];
  std::vector<double> vf = {vxf, vyf};
  return vf;
}
std::vector<double> operator*(std::vector<double> v1, double k) {
  auto vxf = k*v1[0];
  auto vyf = k*v1[1];
  std::vector<double> vf = {vxf, vyf};
  return vf;
}
std::vector<boid> neighbours(boid const& b1, std::vector<boid> const& flock,
                             double d) {
  std::vector<boid> neighbours;
  std::copy_if(flock.begin(), flock.end(), std::back_inserter(neighbours),
               [&b1, d](boid const& b2) { return dist(b1, b2) < d; });
  return neighbours;
}
std::vector<double> alignment(boid const& b1, std::vector<boid> const& flock,
                              double d) {
  std::vector<boid> n = neighbours(b1, flock, d);
  std::vector<double> avg_vel{0, 0};
  for (boid const& b : n) {
    auto vel = b.velocity();
    avg_vel = avg_vel + vel;
  }
  auto size = n.size();
  if (size != 0) {
    avg_vel[0] = avg_vel[0] / size;
    avg_vel[1] = avg_vel[1] / size;
    return avg_vel*3;
  } else {
    return {0, 0};
  }
}

std::vector<double> separation(boid const& b1, std::vector<boid> const& flock,
                               double ds) {
  std::vector<double> sep_vel{0, 0};
  for (boid const& b : flock) {
    if (dist(b1, b) < ds) {
      auto pos = b.position();
      sep_vel[0] = exp(5-(sep_vel[0] - (pos[0] - b1.position()[0])));
      sep_vel[1] = exp(5-(sep_vel[1] - (pos[1] - b1.position()[1])));
    }
  }
  return sep_vel;
}
/*std::vector<double> cohesion(boid const& b1, std::vector<boid> const& flock,
                             double d) {
  std::vector<boid> n = neighbours(b1, flock, d);
  std::vector<double> avg_pos{0, 0};
  for (boid const& b : n) {
    auto pos = b.position();
    avg_pos = avg_pos + pos;
  }
  auto size = n.size();
  if (size != 0) {
    avg_pos[0] = avg_pos[0] / size;
    avg_pos[1] = avg_pos[1] / size;
    return avg_pos;
  } else {
    return {0, 0};
  }
}*/
int main() {
  const int n = 100;
  double d = 100;
  double ds = 1;  // gestire errori di input (mettere catch error)
  double s = 3;   // max vel?
  double a = 2;
  double c = 1;
  double Vmax = 25;
  /* std::cout
        << "inserire nuemero boids, distanza, distanza di separazione, s, a, c.
    "; std::cin >> n >> d >> ds >> s >> a >> c;*/
  int windowWidth = (1) * sf::VideoMode::getDesktopMode().width - 40;
  int windowHeight = (1) * sf::VideoMode::getDesktopMode().height - 75;
  sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight),
                          "Boids Simulation");
  window.setFramerateLimit(144);
  window.setPosition(sf::Vector2i(0, 0));
  // adegurare a framerate tempo e a grandezza boid
  // Assegnazione delle caratteristiche allo spawn dei boid*/
  std::vector<boid> boids;
  for (int i; i < n; i++) {
    std::random_device r;
    std::default_random_engine e1(r());
    std::uniform_int_distribution<> roll_dice1(
        20, windowWidth-20);  // non capisco perchè restituiscono tutti int
                                // (in caso satatic cast)
    int rand_x = roll_dice1(e1);
    std::uniform_int_distribution<> roll_dice2(20, windowHeight - 20);
    int rand_y = roll_dice2(e1);
    std::normal_distribution<> gauss1(0, 0.75);
    int rand_vx = gauss1(e1);
    std::normal_distribution<> gauss2(0, 0.75);
    int rand_vy = gauss2(e1);
    boid bi{rand_x, rand_y, rand_vx, rand_vy};  // implicita conv double int
    boids.push_back(bi);
  }
 
  double sum_dis;

  while (window.isOpen()) {  // un po' buggato sia con opzione schermo intero
                             // che se messo schermo intero dopo
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }
    // update position
    for (boid& b1 : boids) {  // passare const ref
      alignment(b1, boids, d);
      //cohesion(b1, boids, d);
      //separation(b1, boids, ds);
      std::vector<double> lim_vel {0,0};
     //if (b1.position()[0]- windowWidth < 20 ) {
      //lim_vel[0] = exp(3-0.25*(b1.position()[0]- windowWidth));}
      //sep_vel[0] = exp(10-0.5*(unsig(b1.position()[], windowWidth)));}
      std::vector<double> s = b1.position() + alignment(b1, boids, d) + b1.velocity()+ lim_vel;
      b1.setPosition(s);
      for (boid& b2 : boids) {
        sum_dis += dist(b1, b2);
      }
   }
    std::cout << sum_dis / 2 * n << " ";
    window.clear(sf::Color::White);
    for (boid b : boids) {  // passato const& boid
      sf::CircleShape boid_point(1);
      boid_point.setFillColor(sf::Color::Black);   
      auto xy = b.position();
      boid_point.setPosition(xy[0], xy[1]);  // frecce
      window.draw(boid_point);
      sf::Text text;
    }
    // text.setFont(font);
    /*const sf::Color AXIS_COLOR(sf::Color::Black);
    sf::Vertex boid_line[] = {
        sf::Vertex(sf::Vector2f(8, 4), AXIS_COLOR),
        sf::Vertex(sf::Vector2f(150, 150), AXIS_COLOR),
    };
    boid_line.setFillColor(sf::Color::Black);
    boid_line.setPosition(xy[0], xy[1]);
    window.draw(boid_line, 2, sf::Lines);
  }*/
    window.display();
  }
  /*for (boid b : boids) {
    auto velx = b.velocity();
    std::cout << velx[0] << " " << velx[1] << "\n";
  }*/
  return 0;
}
