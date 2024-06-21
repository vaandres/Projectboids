#include "boids.hpp"
#include "operator.hpp"
#include <SFML/Graphics.hpp>
#include <random>

std::array<double, 2> operator+(std::array<double, 2> v1,
                                std::array<double, 2> v2)
{
  auto vxf                 = v1[0] + v2[0];
  auto vyf                 = v1[1] + v2[1];
  std::array<double, 2> vf = {vxf, vyf};
  return vf;
}

std::array<double, 2> operator/(std::array<double, 2> v1, double k)
{
  assert(k != 0);
  auto vxf                 = v1[0] / k;
  auto vyf                 = v1[1] / k;
  std::array<double, 2> vf = {vxf, vyf};
  return vf;
}

int main()
{
  int n       = 200;
  double d    = 90;
  double ds   = 20;  // gestire errori di input (mettere catch error), negativi
  double s    = 0.5; // max vel?
  double a    = 0.1;
  double c    = 0.01;
  double Vmax = 5;
  sf::Font font;
  font.loadFromFile("./Nexa-Heavy.ttf");

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

  // adegurare a framerate tempo e a grandezza boid

  // Assegnazione delle caratteristiche allo spawn dei boid*/
  std::vector<bds::boid> boids;
  for (int i; i < n; i++) {
    std::random_device r;
    std::default_random_engine e1(r());
    std::uniform_real_distribution<> roll_dice1(
        20, windowWidth - 20); // non capisco perchè restituiscono tutti int             //generare con nuova ckasse flock
                               // (in caso satatic cast)
    // int rand_x = roll_dice1(e1);
    std::uniform_real_distribution<> roll_dice2(20, windowHeight - 20);
    // int rand_y = roll_dice2(e1);
    std::normal_distribution<double> gauss1(0, 2);
    // int rand_vx = static_cast<int> (gauss1(e1));
    std::normal_distribution<double> gauss2(0, 2);
    // int rand_vy = static_cast<int> (gauss2(e1));
    bds::boid bi{roll_dice1(e1), roll_dice2(e1), gauss1(e1),
                 gauss2(e1)}; // implicita conv double int
    boids.push_back(bi);
  }
  bds::boid pred {0,0, 3,3};

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

  std::vector<bds::boid> preds = {pred};
    for (bds::boid& b1 : boids) { // passare const ref

      b1.setVelocity(
          b1.velocity() + bds::edgeforce(b1, windowWidth, windowHeight)
          + bds::alignment(b1, boids, d, a) + escape(pred,b1,d,c) + bds::separation(b1, boids, ds, s)/* separation(b1,preds,d,c) */ //fix con escape
          + bds::cohesion(b1, boids, d, c));
      bds::velocitylimit(b1, Vmax);
      std::array<double, 2> p = b1.position() + b1.velocity();
      b1.setPosition(p);
      assert(b1.position()[0] <= windowWidth + 100);
      assert(b1.position()[1] <= windowHeight + 100);
      assert(b1.position()[0] >= - 100);
      assert(b1.position()[1] >= - 100);
    }
  /* std::vector<bds::boid> prey = neighbours(pred,boids,d);
  for (bds::boid& b1: prey){
    b1.setVelocity(b1.velocity()+ escape(pred,b1,d,c));
  } */
   pred.setVelocity(pred.velocity()+  bds::follow(pred,boids,d,Vmax) +bds::edgeforce(pred, windowWidth, windowHeight));
   bds::velocitylimit(pred, Vmax);
   std::array<double, 2> p = pred.position() + pred.velocity();
   pred.setPosition(p);

    window.clear(sf::Color::White);
    for (bds::boid b : boids) { // passato const& boid
      sf::CircleShape boid_point(2);
      boid_point.setFillColor(sf::Color::Black);
      auto xy = b.position();
      boid_point.setPosition(
          static_cast<float>(xy[0]),
          static_cast<float>(xy[1])); // frecce /è necessari static cast?
      window.draw(boid_point);
    }
    sf::CircleShape pred_point(4);
    pred_point.setFillColor(sf::Color::Red);
    auto xy = pred.position();
    pred_point.setPosition(
      static_cast<float>(xy[0]),
          static_cast<float>(xy[1])); 
      window.draw(pred_point);

    window.display();

    if (window2.isOpen()) {
      //   statistics data = stats(boids);
      //   window2.clear(sf::Color::White);
      //   sf::Text text;
      //   text.setFont(font);
      //   text.setString(
      //       "Avarage velocity" + std::to_string(data.vel_mean[0]) + "   "
      //       + std::to_string(data.vel_mean[1]) + "\n\n"
      //       + "Standard deviation: " + std::to_string(data.vel_sigma[0]) + "
      //       "
      //       + std::to_string(data.vel_sigma[1]) + "\n\n"
      //       + "Avarage distance: " + std::to_string(data.dis_mean) + "\n\n"
      //       + "Standard deviation: " + std::to_string(data.dis_sigma));
      //   text.setCharacterSize(7);
      //   text.setFillColor(sf::Color::Black);
      //   text.setPosition(5, 5);
      //   window2.draw(text);
      //   window2.display();
      //   // text.setFont(font);
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
}