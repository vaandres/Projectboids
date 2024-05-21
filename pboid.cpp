#include <math.h> 
#include <SFML/Graphics.hpp>
#include <cmath>
#include <iostream>
#include <random>
#include <array>

class boid {  // Costruttore della classe boids
 private:
  std::array<double,2> position_{0, 0};  // struct personalizzata
  std::array<double,2> velocity_{0, 0};

 public:
  boid(double x, double y, double vx, double vy)
      : position_{x, y}, velocity_{vx, vy} {}

  std::array<double,2> position() const { return position_; }

  std::array<double,2> velocity() const { return velocity_; }
  void setPosition(const std::array<double,2>& newPos) { position_ = newPos; }
  void setVelocity(const std::array<double,2>& newVel) { velocity_ = newVel; }
};

struct statistics {
  double dis_mean;
  double dis_sigma;
  std::array<double,2> vel_mean;
  std::array<double,2> vel_sigma;
};
std::array<double,2> operator+(std::array<double,2> v1, std::array<double,2> v2) {
  auto vxf = v1[0] + v2[0];
  auto vyf = v1[1] + v2[1];
  std::array<double,2> vf = {vxf, vyf};
  return vf;
}
std::array<double,2> operator-(std::array<double,2> v1, std::array<double,2> v2) {
  auto vxf = v1[0] - v2[0];
  auto vyf = v1[1] - v2[1];
  std::array<double,2> vf = {vxf, vyf};
  return vf;
}
std::array<double,2> operator/(std::array<double,2> v1, double k) {
  auto vxf = v1[0] / k;
  auto vyf = v1[1] / k;
  std::array<double,2> vf = {vxf, vyf};
  return vf;
}
std::array<double,2> operator*(std::array<double,2> v1, std::array<double,2> v2) {
  auto vxf = v2[0] * v1[1];
  auto vyf = v2[0] * v1[1];
  std::array<double,2> vf = {vxf, vyf};
  return vf;
}

double dist(boid const& b1, boid const& b2) {
  auto pos1 = b1.position();
  auto pos2 = b2.position();
  return std::sqrt(
      std::pow(pos1[0] - pos2[0], 2) +  // conservare velocità totale
      std::pow(pos1[1] - pos2[1], 2));
}

statistics stats(std::vector<boid> const& flock) {
  double dis_mean{0.0};
  double dis_sigma{0.0};
  int n = flock.size();
  std::array<double,2> vel_mean;
  std::array<double,2> vel_sigma;

  for (boid const& b1 : flock) {
    // pos_mean = ;
    vel_mean = vel_mean + b1.velocity();
    for (boid const& b2 : flock) {
      dis_mean = dis_mean + dist(b1, b2);
    }
  }

  vel_mean = vel_mean / n;
  dis_mean = 0.0264583333 * dis_mean / (2 * n);
  for (boid const& b1 : flock) {
    for (boid const& b2 : flock) {
      dis_sigma = dis_sigma + (dis_mean - 0.0264583333 * dist(b1, b2)) *
                                  (dis_mean - 0.0264583333 * dist(b1, b2));
    }
  }
  for (boid const& b : flock) {
    vel_sigma =
        vel_sigma + (vel_mean - b.velocity()) * (vel_mean - b.velocity());
  }
  dis_sigma = std::sqrt(fabs(dis_sigma) / (n - 1));
  vel_sigma[0] = std::sqrt(fabs(vel_sigma[0]) / (n - 1));
  vel_sigma[1] = std::sqrt(fabs(vel_sigma[1]) / (n - 1));
  return {dis_mean, dis_sigma, vel_mean, vel_sigma};
}

std::array<double,2> operator/(std::array<double,2> v1, std::array<double,2> v2) {
  auto vxf = v1[0] / v2[0];  // errore se v2[0] = 0 ecc
  auto vyf = v1[1] / v2[1];
  std::array<double,2> vf = {vxf, vyf};
  return vf;
}

std::array<double,2> operator*(std::array<double,2> v1, double k) {
  auto vxf = k * v1[0];
  auto vyf = k * v1[1];
  std::array<double,2> vf = {vxf, vyf};
  return vf;
}

std::vector<boid> neighbours(boid const& b1, std::vector<boid> const& flock,
                             double d) {
  std::vector<boid> neighbours;
  std::copy_if(flock.begin(), flock.end(), std::back_inserter(neighbours),
               [&b1, d](boid const& b2) { return dist(b1, b2) < d; });
  return neighbours;
}
std::array<double,2> alignment(boid const& b1, std::vector<boid> const& flock,
                              double d) {
  std::vector<boid> n = neighbours(b1, flock, d);
  std::array<double,2> avg_vel{0, 0};
  for (boid const& b : n) {
    auto vel = b.velocity();
    avg_vel = avg_vel + vel;
  }
  auto size = n.size();
  if (size != 0) {
    avg_vel[0] = avg_vel[0] / size;
    avg_vel[1] = avg_vel[1] / size;
    return avg_vel;
  } else {
    return {0, 0};
  }
}

std::array<double,2> separation(boid const& b1, std::vector<boid> const& flock,
                               double ds) {
  std::array<double,2> sep_vel{0, 0};
  for (boid const& b : flock) {
    if (dist(b1, b) < ds) {
      auto pos = b.position();
      sep_vel[0] =
          sep_vel[0] - (pos[0] - b1.position()[0]) / ds;  // numero da 0 a 1
      sep_vel[1] = sep_vel[1] - (pos[1] - b1.position()[1]) / ds;
    }
  }
  return sep_vel;
}
std::array<double,2> cohesion(boid const& b1, std::vector<boid> const& flock,
                             double d) {
  std::vector<boid> n = neighbours(b1, flock, d);
  std::array<double,2> avg_pos{0, 0};
  for (boid const& b : n) {
    auto pos = b.position();
    avg_pos = avg_pos + pos;
  }
  auto size = n.size();
  if (size != 0) {
    avg_pos = avg_pos / size;
    avg_pos = (avg_pos - b1.position()) * 0.02;
    return avg_pos;
  } else {
    return {0, 0};
  }
}
// forza di repulsione dal bordo; è del tipo sgn(x-a)/(abs(x^2-a)-a)^2 dal
// grafico si capisce perché
std::array<double,2> edgeforce(boid const& b, int width, int height) {
  double x{b.position()[0]};
  double y = {b.position()[1]};
  double or_x = width / 2;
  double or_y = height / 2;
  double ax =
      -1 / std::pow(std::abs(x - or_x) - or_x, 2) * abs(x - or_x) / (x - or_x);
  double ay =
      -1 / std::pow(std::abs(y - or_y) - or_y, 2) * abs(y - or_y) / (y - or_y);
  std::array<double,2> v = {ax, ay};
  return v;
}
void velocitylimit(boid& b, double Vmax) {
  if (b.velocity()[0] > Vmax / std::sqrt(2)) {
    std::array<double,2> vel{Vmax / std::sqrt(2), vel[1]};
    b.setVelocity(vel);
  }
  if (b.velocity()[1] > Vmax / std::sqrt(2)) {
    std::array<double,2> vel{vel[0], Vmax / std::sqrt(2)};  // fare meglio
    b.setVelocity(vel);
  }
  if (b.velocity()[0] < -Vmax / std::sqrt(2)) {
    std::array<double,2> vel{-Vmax / std::sqrt(2), vel[1]};
    b.setVelocity(vel);
  }
  if (b.velocity()[1] < -Vmax / std::sqrt(2)) {
    std::array<double,2> vel{vel[0], -Vmax / std::sqrt(2)};
    b.setVelocity(vel);
  }
}

int main() {
  /* std::cout
       << "inserire nuemero boids, distanza, distanza di separazione, s, "  //
     mettere
                                                                            //
     slider
                                                                            //
     per
                                                                            //
     cambiare
                                                                            //
     in
                                                                            //
     tempo
                                                                            //
     reale "a, c,Vmax<."<<"/n";*/
  int n = 160;
  double d = 150;
  double ds = 10;  // gestire errori di input (mettere catch error)
  double s = 3;    // max vel?
  double a = 2;
  double c = 1;
  double Vmax = 4;  // idealmente componenti sqrt(vmax^2/2) = vmax/sqrt2
  /*std::cin >> n >> d >> ds >> s >> a >> c >> Vmax;*/
  sf::Font font;
  font.loadFromFile("./Nexa-Heavy.ttf");
  /*if (!font.loadFromFile("arial.ttf")) {   "catch error" suggeritoda copilot
    std::cerr << "Could not load font\n";
    return 1;
  }*/
  /* std::cout
        << "inserire nuemero boids, distanza, distanza di separazione, s, a, c.
    "; std::cin >> n >> d >> ds >> s >> a >> c;*/
  int windowWidth = (1) * sf::VideoMode::getDesktopMode().width - 40;
  int windowHeight = (1) * sf::VideoMode::getDesktopMode().height - 75;
  sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight),
                          "Boids Simulation");
  window.setFramerateLimit(60);
  window.setPosition(sf::Vector2i(0, 0));
  // finestra statistiche
  sf::RenderWindow window2(sf::VideoMode(200, 200), "Statistics");
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
    std::normal_distribution<> gauss1(0, 2);
    int rand_vx = gauss1(e1);
    std::normal_distribution<> gauss2(0, 2);
    int rand_vy = gauss2(e1);
    boid bi{rand_x, rand_y, rand_vx, rand_vy};  // implicita conv double int
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
    // update position
    // double sum_dis{0};
    for (boid& b1 : boids) {  // passare const ref
      alignment(b1, boids, d);
      cohesion(b1, boids, d);
      separation(b1, boids, ds);
      b1.setVelocity(
          b1.velocity() +
          edgeforce(b1, windowWidth, windowHeight));  // edgeforce qui o sotto?
      std::array<double,2> s =
          b1.position() + (alignment(b1, boids, d) * 5 + b1.velocity()) / 3 +
          (separation(b1, boids, ds) * 3 + cohesion(b1, boids, d)) / 2;
      velocitylimit(b1, Vmax);
      b1.setPosition(s);
      /* for (boid& b2 : boids) {
         sum_dis += dist(b1, b2);
       }*/
    }
    // std::cout << sum_dis / 2 * n << " ";
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
    if (window2.isOpen()) {
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
      // text.setFont(font);
      /*const sf::Color AXIS_COLOR(sf::Color::Black);
      sf::Vertex boid_line[] = {
          sf::Vertex(sf::array2f(8, 4), AXIS_COLOR),
          sf::Vertex(sf::array2f(150, 150), AXIS_COLOR),
      };
      boid_line.setFillColor(sf::Color::Black);
      boid_line.setPosition(xy[0], xy[1]);
      window.draw(boid_line, 2, sf::Lines);
    }*/
    }
  }

  /*for (boid b : boids) {
    auto velx = b.velocity();
    std::cout << velx[0] << " " << velx[1] << "\n";
  }*/
  return 0;
}








/*int main() {
  int n = 10;
  double d = 5;
  double ds = 4;  // gestire errori di input (mettere catch error)
  double s = 3;   // max vel?
  double a = 2;
  double c = 1;
   std::cout
        << "inserire nuemero boids, distanza, distanza di separazione, s, a, c.
    "; std::cin >> n >> d >> ds >> s >> a >> c;
  int windowWidth = (1) * sf::VideoMode::getDesktopMode().width - 40;
  int windowHeight = (1) * sf::VideoMode::getDesktopMode().height - 75;
  sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight),
                          "Boids Simulation");
  window.setFramerateLimit(144);
  window.setPosition(sf::Vector2i(0, 0));
  // adegurare a framerate tempo e a grandezza boid
  // Assegnazione delle caratteristiche allo spawn dei boid
  // !
  for (int i; i < n; i++) {
  }
  while (window.isOpen()) {  // un po' buggato sia con opzione schermo intero
                             // che se messo schermo intero dopo
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      }
    }
    window.clear(sf::Color::White);
    sf::CircleShape shape(1);
    shape.setFillColor(sf::Color::Black);
    shape.setPosition(50, 50);
    window.draw(shape);

    window.display();
  }
  return 0;
}*/
