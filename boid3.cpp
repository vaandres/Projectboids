#include <math.h>  //quale

#include <SFML/Graphics.hpp>
#include <array>
#include <cmath>
#include <iostream>
#include <random>

class boid {  // Costruttore della classe boids
 private:
  std::array<double, 3> position_{0, 0, 0};  // struct personalizzata
  std::array<double, 3> velocity_{0, 0, 0};

 public:
  boid(double x, double y, double z, double vx, double vy, double vz)
      : position_{x, y, z}, velocity_{vx, vy, vz} {}

  std::array<double, 3> position() const { return position_; };

  std::array<double, 3> velocity() const { return velocity_; };
  void setPosition(const std::array<double, 3>& newPos) { position_ = newPos; }
  void setVelocity(const std::array<double, 3>& newVel) { velocity_ = newVel; }
};

struct statistics {
  double dis_mean;
  double dis_sigma;
  std::array<double, 3> vel_mean;
  std::array<double, 3> vel_sigma;
};
std::array<double, 3> operator+(std::array<double, 3> v1,
                                std::array<double, 3> v2) {
  auto vxf = v1[0] + v2[0];
  auto vyf = v1[1] + v2[1];
  auto vzf = v1[2] + v2[2];
  std::array<double, 3> vf = {vxf, vyf, vzf};
  return vf;
}
std::array<double, 3> operator-(std::array<double, 3> v1,
                                std::array<double, 3> v2) {
  auto vxf = v1[0] - v2[0];
  auto vyf = v1[1] - v2[1];
    auto vzf = v1[2] - v2[2];
  std::array<double, 3> vf = {vxf, vyf, vzf};
  return vf;
}
std::array<double, 3> operator/(std::array<double, 3> v1, double k) {
  auto vxf = v1[0] / k;
  auto vyf = v1[1] / k;
  auto vzf = v1[2] / k;
  std::array<double, 3> vf = {vxf, vyf, vzf};
  return vf;
}
std::array<double, 3> operator*(std::array<double, 3> v1,
                                std::array<double, 3> v2) {
  auto vxf = v2[0] * v1[1];
  auto vyf = v2[0] * v1[1];
  auto vzf = v2[0] * v1[2];
  std::array<double, 3> vf = {vxf, vyf, vzf};
  return vf;
}

double dist(boid const& b1, boid const& b2) {
  auto pos1 = b1.position();
  auto pos2 = b2.position();
  return std::sqrt(
      std::pow(pos1[0] - pos2[0], 2) +  // conservare velocità totale
      std::pow(pos1[1] - pos2[1], 2)+ std::pow(pos1[2]-pos2[2],2));  // attrazione al centro
}

std::array<double, 3> operator/(std::array<double, 3> v1,
                                std::array<double, 3> v2) {
  auto vxf = v1[0] / v2[0];  // errore se v2[0] = 0 ecc
  auto vyf = v1[1] / v2[1];
  auto vzf = v1[2] / v2[2];
  std::array<double, 3> vf = {vxf, vyf,vzf};
  return vf;
}

std::array<double, 3> operator*(std::array<double, 3> v1, double k) {
  auto vxf = k * v1[0];
  auto vyf = k * v1[1];
  auto vzf = k * v1[2];
  std::array<double, 3> vf = {vxf, vyf,vzf};
  return vf;
}

std::vector<boid> neighbours(boid const& b1, std::vector<boid> const& flock,
                             double d) {
  std::vector<boid> neighbours;
  std::copy_if(flock.begin(), flock.end(), std::back_inserter(neighbours),
               [&b1, d](boid const& b2) { return dist(b1, b2) < d; });
  return neighbours;
}
std::array<double, 3> alignment(boid const& b1, std::vector<boid> const& flock,
                                double d, double c) {
  std::vector<boid> n = neighbours(b1, flock, d);
  std::array<double, 3> avg_vel{0, 0,0};
  for (boid const& b : n) {
    auto vel = b.velocity();
    avg_vel = avg_vel + vel;
  }
  auto size = n.size();
  if (size != 0) {
    avg_vel[0] = avg_vel[0] / size;
    avg_vel[1] = avg_vel[1] / size;
    avg_vel[2] = avg_vel[2] / size;
    return (avg_vel - b1.velocity()) * c;
  } else {
    return {0, 0};
  }
}

std::array<double, 3> separation(boid const& b1, std::vector<boid> const& flock,
                                 double ds, double s) {
  std::array<double, 3> sep_vel{0, 0,0};
  for (boid const& b : flock) {
    if (dist(b1, b) < ds) {
      auto pos = b.position();
      sep_vel[0] =
          sep_vel[0] - (pos[0] - b1.position()[0]) / ds;  // numero da 0 a 1
      sep_vel[1] = sep_vel[1] - (pos[1] - b1.position()[1]) / ds;
      sep_vel[2] = sep_vel[2] - (pos[2] - b1.position()[2]) / ds;
    }
  }
  return sep_vel * s;
}

std::array<double, 3> cohesion(boid const& b1, std::vector<boid> const& flock,
                               double d, double s) {
  std::vector<boid> n = neighbours(b1, flock, d);
  std::array<double, 3> avg_pos{0, 0};
  for (boid const& b : n) {
    auto pos = b.position();
    avg_pos = avg_pos + pos;
  }
  auto size = n.size();
  if (size != 0) {
    avg_pos = avg_pos / size;
    avg_pos = (avg_pos - b1.position()) * s;
    return avg_pos;
  } else {
    return {0, 0};
  }
}

std::array<double, 3> center_force(boid const& b, boid const& center) {
    auto center_force =(center.position()-b.position())/dist(b,center);
    return center_force * 0.05;
}

std::array<double, 3> edgeforce(boid const& b, int width, int height, int depth) {
  double x{b.position()[0]};
  double y{b.position()[1]};
  double z{b.position()[2]};

  double vx{0};
  double vy{0};
  double vz{0};

  if (x <= 5) {
    vx = 1;
  } else if (x >= width - 5) {
    vx = -1;
  }

  if (y <= 5) {
    vy = 1;
  } else if (y >= height - 5) {
    vy = -1;
  }
  if (z <= 5) {
    vz = 1;
  } else if (z >= depth - 5) {
    vz = -1;
  }

  std::array<double, 3> a{vx, vy};
  return a;
}
void velocitylimit(boid& b, double Vmax) {
  if (b.velocity()[0] > Vmax / std::sqrt(2)) {
    std::array<double, 3> vel{Vmax / std::sqrt(2), vel[1]};
    b.setVelocity(vel);
  }
  if (b.velocity()[1] > Vmax / std::sqrt(2)) {
    std::array<double, 3> vel{vel[0], Vmax / std::sqrt(2)};  // fare meglio
    b.setVelocity(vel);
  }
  if (b.velocity()[0] < -Vmax / std::sqrt(2)) {
    std::array<double, 3> vel{-Vmax / std::sqrt(2), vel[1]};
    b.setVelocity(vel);
  }
  if (b.velocity()[1] < -Vmax / std::sqrt(2)) {
    std::array<double, 3> vel{vel[0], -Vmax / std::sqrt(2)};
    b.setVelocity(vel);
  }
  if (b.velocity()[2] > Vmax / std::sqrt(2)) {
    std::array<double, 3> vel{vel[0], vel[1], Vmax / std::sqrt(2)};
    b.setVelocity(vel);
  }
    if (b.velocity()[2] < -Vmax / std::sqrt(2)) {
        std::array<double, 3> vel{vel[0], vel[1], -Vmax / std::sqrt(2)};
        b.setVelocity(vel);
    }
}

int main() {
  int n = 160;
  double d = 90;
  double ds = 20;    // gestire errori di input (mettere catch error), negativi
  double s = 0.001;  // max vel?
  double a = 0.5;
  double c = 0.15;
  int depth = 100;
  boid center = {sf::VideoMode::getDesktopMode().width/2,sf::VideoMode::getDesktopMode().height/2,depth/2,0,0,0};
  double Vmax = 4;  // idealmente componenti sqrt(vmax^2/2) = vmax/sqrt2
  /*std::cin >> n >> d >> ds >> s >> a >> c >> Vmax;*/
  sf::Font font;
  font.loadFromFile("./Nexa-Heavy.ttf");

  int windowWidth = (1) * sf::VideoMode::getDesktopMode().width - 40;
  int windowHeight = (1) * sf::VideoMode::getDesktopMode().height - 75;
  sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight),
                          "Boids Simulation");
  window.setFramerateLimit(60);
  window.setPosition(sf::Vector2i(0, 0));
  // finestra statistiche
  sf::RenderWindow window2(sf::VideoMode(200, 300), "Statistics");
  window2.setFramerateLimit(30);
  window2.setPosition(sf::Vector2i(windowWidth - 220, windowHeight - 200));
  // adegurare a framerate tempo e a grandezza boid
  // Assegnazione delle caratteristiche allo spawn dei boid*/
  std::vector<boid> boids;
  for (int i; i < n; i++) {
    std::random_device r;
    std::default_random_engine e1(r());
    std::uniform_int_distribution<> roll_dice1(
        20, windowWidth - 20);  // non capisco perchè restituiscono tutti int
                                // (in caso satatic cast)
    int rand_x = roll_dice1(e1);
    std::uniform_int_distribution<> roll_dice2(20, windowHeight - 20);
    int rand_y = roll_dice2(e1);
    std::uniform_int_distribution<> roll_dice3(20, windowHeight - 20);
    int rand_z = roll_dice2(e1);
    std::normal_distribution<> gauss1(0, 3);
    int rand_vx = gauss1(e1);
    std::normal_distribution<> gauss2(0, 3);
    int rand_vy = gauss2(e1);
    std::normal_distribution<> gauss3(0, 3);
    int rand_vz = gauss2(e1);
    boid bi{rand_x,  rand_y,  rand_z,
            rand_vx, rand_vy, rand_vz};  // implicita conv double int
    boids.push_back(bi);
  }

  while (window.isOpen() |
         window2.isOpen()) {  // un po' buggato sia con opzione schermo intero
                              // che se messo schermo intero dopo
                              // dividere while per finestre
    sf::Event event;
    sf::Event event2;
    while (window.pollEvent(event) | window2.pollEvent(event2)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
      if (event2.type == sf::Event::Closed) {
        window2.close();
      }
    }

    for (boid& b1 : boids) {  // passare const ref
      alignment(b1, boids, d, c);
      cohesion(b1, boids, d, s);
      separation(b1, boids, ds, a);
      b1.setVelocity(b1.velocity() + edgeforce(b1, windowWidth, windowHeight, depth) +
                     alignment(b1, boids, d, c) + separation(b1, boids, ds, a) +
                     cohesion(b1, boids, d, s)+ center_force(b1,center));  // edgeforce qui o sotto?
      std::array<double, 3> p = b1.position() + b1.velocity();
      velocitylimit(b1, Vmax);
      b1.setPosition(p);
    }
    window.clear(sf::Color::White);
    for (boid b : boids) {  // passato const& boid
      sf::CircleShape boid_point(1);
      boid_point.setFillColor(sf::Color::Black);
      auto xy = b.position();
      boid_point.setPosition(xy[0], xy[1]);  // frecce
      window.draw(boid_point);
    }

    window.display();
    // 0.0264583333 *
    /*if (window2.isOpen()) {
      statistics data = stats(boids);
      window2.clear(sf::Color::White);
      sf::Text text;
      text.setFont(font);
      text.setString(
          "Avarage velocity" + std::to_string(data.vel_mean[0]) + "   " +
          std::to_string(data.vel_mean[1]) + "\n\n" +
          "Standard deviation: " + std::to_string(data.vel_sigma[0]) + "   " +
          std::to_string(data.vel_sigma[1]) + "\n\n" +
          "Avarage distance: " + std::to_string(data.dis_mean) + "\n\n" +
          "Standard deviation: " + std::to_string(data.dis_sigma));
      text.setCharacterSize(7);
      text.setFillColor(sf::Color::Black);
      text.setPosition(5, 5);
      window2.draw(text);
      window2.display();
    }*/
  }
  return 0;
}