#include "boids.hpp"
#include "operator.hpp"
#include <SFML/Graphics.hpp>
#include <random>

int main()
{
  int n{100};
  double d{90};
  double ds{15}; // gestire errori di input (mettere catch error), negativi
  double s{0.4}; // max vel?
  double a{0.1};
  double c{0.01};
  double e{3.};
  double f{0.4};
  double Vmax{8};
  const double range{7};
  const double pred_coeff{1.05};
  sf::Font font;
  font.loadFromFile("../Nexa-Heavy.ttf");

  unsigned windowWidth  = (1) * sf::VideoMode::getDesktopMode().width - 40;
  unsigned windowHeight = (1) * sf::VideoMode::getDesktopMode().height - 75;
  sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight),
                          "Boids Simulation");
  window.setFramerateLimit(30);
  window.setPosition(sf::Vector2i(0, 0));

  // finestra statistiche
  sf::RenderWindow window2(sf::VideoMode(200, 200), "Statistics");
  window2.setFramerateLimit(30);
  window2.setPosition(sf::Vector2i(static_cast<int>(windowWidth) - 220,
                                   static_cast<int>(windowHeight) - 200));
  //????Esisite modo diverso invece di usare static cast.

  // adegurare a framerate tempo e a grandezza Boid

  // Assegnazione delle caratteristiche allo spawn dei Boid*/
  std::vector<bds::Boid> flock;
  for (int i; i < n; i++) {
    std::random_device r;
    std::default_random_engine e1(r());
    std::uniform_real_distribution<> roll_dice1(
        20, windowWidth - 20); // non capisco perchè restituiscono tutti int
                               // (in caso satatic cast)
    // int rand_x = roll_dice1(e1);
    std::uniform_real_distribution<> roll_dice2(20, windowHeight - 20);
    // int rand_y = roll_dice2(e1);
    std::normal_distribution<double> gauss1(0, 2);
    // int rand_vx = static_cast<int> (gauss1(e1));
    std::normal_distribution<double> gauss2(0, 2);
    // int rand_vy = static_cast<int> (gauss2(e1));
    bds::Boid bi{roll_dice1(e1), roll_dice2(e1), gauss1(e1),
                 gauss2(e1)}; // implicita conv double int
    flock.push_back(bi);
  }

  bds::Boid pred{0, 0, 3, 3};

  while (window.isOpen()
         | window2.isOpen()) { // un po' buggato sia con opzione schermo intero
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

    for (bds::Boid& b1 : flock) {
      bds::applyRules(b1, a, c, s, d, ds, e, windowWidth, windowHeight, flock,
                      pred);
      bds::velocitylimit(b1, Vmax);
      b1.updatePosition();
      assert(b1.position()[0] <= windowWidth + 100);
      assert(b1.position()[1] <= windowHeight + 100);
      assert(b1.position()[0] >= -100);
      assert(b1.position()[1] >= -100);
    }

    bds::RulesPred(pred, flock, f, windowWidth, windowHeight);
    bds::velocitylimit(pred, Vmax * pred_coeff);
    pred.updatePosition();
    bds::eat(pred, flock, range);

    window.clear(sf::Color::White);
    for (bds::Boid& b : flock) { // passato const& Boid
      sf::CircleShape Boid_point(2);
      Boid_point.setFillColor(sf::Color::Black);
      auto xy = b.position();
      Boid_point.setPosition(
          static_cast<float>(xy[0]),
          static_cast<float>(xy[1])); // frecce /è necessari static cast?
      window.draw(Boid_point);
    }

    sf::CircleShape pred_point(4);
    pred_point.setFillColor(sf::Color::Red);
    auto xy = pred.position();
    pred_point.setPosition(static_cast<float>(xy[0]),
                           static_cast<float>(xy[1]));
    window.draw(pred_point);

    window.display();

    if (window2.isOpen()) {
      bds::Statistics data = bds::stats(flock);
      window2.clear(sf::Color::White);
      sf::Text text;
      text.setFont(font);
      text.setString(
          "Avarage velocity" + std::to_string(data.speed_mean) + /* + "   "
           + std::to_string(data.vel_mean[1]) + */
          "\n\n"
          + "Standard deviation: " + std::to_string(data.speed_err) /* + "
          "
         + std::to_string(data.vel_sigma[1])  */
          + "\n\n" + "Avarage distance: " + std::to_string(data.dis_mean)
          + "\n\n" + "Standard deviation: " + std::to_string(data.dis_err));
      text.setCharacterSize(7);
      text.setFillColor(sf::Color::Black);
      text.setPosition(5, 5);
      window2.draw(text);
      window2.display();
      //  text.setFont(font);
      /*const sf::Color AXIS_COLOR(sf::Color::Black);
      sf::Vertex Boid_line[] = {
          sf::Vertex(sf::array2f(8, 4), AXIS_COLOR),
          sf::Vertex(sf::array2f(150, 150), AXIS_COLOR),
      };
      Boid_line.setFillColor(sf::Color::Black);
      Boid_line.setPosition(xy[0], xy[1]);
      window.draw(Boid_line, 2, sf::Lines);
    }*/
    }
  }
}
