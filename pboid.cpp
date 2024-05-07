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
      : position_{x, y}, velocity_{vx, vy} {}

  std::vector<double> position(){return position_;};

  std::vector<double> velocity(){return velocity_;};
};
int main() {
  int n = 10000;
  double d = 5;
  double ds = 4;  // gestire errori di input (mettere catch error)
  double s = 3;   // max vel?
  double a = 2;
  double c = 1;
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
  // Assegnazione delle caratteristiche allo spawn dei boid
  std::vector<boid> boids;
  for (int i; i < n; i++) {  
    std::random_device r;
    std::default_random_engine e1(r());
    std::uniform_int_distribution<> roll_dice1(20, windowWidth - 20);
    int rand_x = roll_dice1(e1);
    std::uniform_int_distribution<> roll_dice2(20, windowHeight - 20);
    int rand_y = roll_dice2(e1);
    // int rand_vx =
    // int rand_vy=
    boid bi{rand_x, rand_y, 0, 0};  //implicita conv double int
    boids.push_back(bi);
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
    for (boid b: boids) {  //passato const& boid
      sf::CircleShape shape(1);  
      shape.setFillColor(sf::Color::Black);
      auto xy = b.position();
      shape.setPosition(xy[0],xy[1]);
      window.draw(shape);
    }
    window.display();
  }
  return 0;
}
