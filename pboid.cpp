#include <SFML/Graphics.hpp>
#include <cmath>  
#include <iostream>
#include <vector>

int main() {
  int n = 10;
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
}
