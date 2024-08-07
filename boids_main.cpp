#include "operator.hpp"
#include <SFML/Graphics.hpp>
#include <iostream>
#include <random>

int main()
{
  try {
    // parametri in input
    bool cin_on{false};
    int n{100};
    double d{90};
    double ds{15};
    double s{0.5};
    double a{0.09};
    double c{0.005};
    bool Predator_on{true};
    double e=(Predator_on)?2:0;
    double f{2.5};
    double pred_coeff{1.3};
    double const range{8};                 //[NON MODIFICARE]
    double const Vmax{10 / bds::conv_fac}; // non cambiare
    // Lettura dei parametri con cin
    if (cin_on) {
      std::cout << "Scegliere la modalità con (1) o senza predatore (0): \n";
      std::cin >> Predator_on;
      if (Predator_on == true) {
        std::cout << "Inserire in ordine : numero di boids , d , ds , s , a , "
                     "c , e , f , pred_coeff \n";
        std::cin >> n >> d >> ds >> s >> a >> c >> e >> f >> pred_coeff;
      } else {
        std::cout
            << "Inserire in ordine : numero di boids , d , ds , s , a , c \n";
        std::cin >> n >> d >> ds >> s >> a >> c;
        e = 0;
      }
      if (std::cin.fail()) {
        throw std::invalid_argument(
            "Input non valido. Si prega di inserire i valori corretti.");
      }

      if (n < 0 || d < 0 || ds < 0 || s < 0 || a < 0 || c < 0 || e < 0 || f < 0
          || pred_coeff < 0) {
        throw std::invalid_argument(
            "Input non valido. Si prega di inserire i valori corretti.");
      }
    }

    // selezione del font per la scrittura delle statistiche dei boids
    sf::Font font;
    font.loadFromFile("./Nexa-Heavy.ttf");

    // creazione della finestra di sfml grafica
    unsigned int windowWidth = (1) * sf::VideoMode::getDesktopMode().width - 40;
    unsigned int windowHeight =
        (1) * sf::VideoMode::getDesktopMode().height - 75;
    sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight),
                            "Boids Simulation");
    window.setFramerateLimit(bds::framerate);
    window.setPosition(sf::Vector2i(0, 0));

    // creazione della finestra delle statistiche
    sf::RenderWindow window2(sf::VideoMode(200, 200), "Statistics");
    window2.setFramerateLimit(30);
    window2.setPosition(sf::Vector2i(static_cast<int>(windowWidth) - 220,
                                     static_cast<int>(windowHeight) - 200));

    // generazione delle variabili per lo spawn dei boids e predatore
    std::random_device r;
    std::default_random_engine eng(r());
    std::vector<bds::Boid> flock(
        static_cast<std::vector<bds::Boid>::size_type>(n));
    std::uniform_real_distribution<> roll_diceX(20, windowWidth - 20);
    std::uniform_real_distribution<> roll_diceY(20, windowHeight - 20);
    std::uniform_real_distribution<> roll_diceVx_boid(-Vmax / 2, Vmax / 2);
    std::uniform_real_distribution<> roll_diceVy_boid(-Vmax / 2, Vmax / 2);
    std::uniform_real_distribution<> roll_diceVx_pred(-Vmax * pred_coeff / 2,
                                                      Vmax * pred_coeff / 2);
    std::uniform_real_distribution<> roll_diceVy_pred(-Vmax * pred_coeff / 2,
                                                      Vmax * pred_coeff / 2);

    // loop di generazione dei boids

    auto generate_boid = [&]() {
      bds::Boid boid{roll_diceX(eng), roll_diceY(eng), roll_diceVx_boid(eng),
                     roll_diceVy_boid(eng)};

      assert(boid.get_position().x <= windowWidth);
      assert(boid.get_position().y <= windowHeight);
      assert(boid.get_position().x >= 0);
      assert(boid.get_position().y >= 0);
      assert(boid.absolute_velocity() <= Vmax);

      return boid;
    };

    std::generate(flock.begin(), flock.end(), generate_boid);
    std::vector<bds::Boid> copy_of_flock{flock};

    // generazione del predatoratore
    bds::Boid predator{roll_diceX(eng), roll_diceY(eng), roll_diceVx_pred(eng),
                       roll_diceVy_pred(eng)};

    assert(predator.get_position().x <= windowWidth);
    assert(predator.get_position().y <= windowHeight);
    assert(predator.get_position().x >= 0);
    assert(predator.get_position().y >= 0);

    assert(predator.absolute_velocity() <= Vmax * pred_coeff);

    // inizio gameloop di sfml
    while (window.isOpen() | window2.isOpen()) {
      // chiusura delle finestre
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

      // loop di cambio di posizione dei boids. La velocità dei boids viene
      // modificata con la funzione apply_rules_boids, successivamente è
      // limitata con velocity_limit ed in infine la posizione è aggiornata con
      // updatePosition
      std::for_each(flock.begin(), flock.end(), [&](bds::Boid& boid_i) {
        apply_rules(
            boid_i, a, c, s, d, ds, e, windowWidth, windowHeight, copy_of_flock,
            predator); // da sommare poi a tutte le velocita e poi undate pos
        /*        bds::apply_rules_boids(boid_i, a, c, s, d, ds, e, windowWidth,
                                      windowHeight, flock, predator); */

        bds::velocity_limit(boid_i, Vmax); // va dopo
        assert(boid_i.absolute_velocity() <= Vmax + 0.0001);

        boid_i.update_position();

        // boid_i.update_position(); // va dopo
        assert(boid_i.get_position().x <= windowWidth);
        assert(boid_i.get_position().y <= windowHeight);
        assert(boid_i.get_position().x >= 0);
        assert(boid_i.get_position().y >= 0);
      });

      copy_of_flock = flock;

      // cambio di posizione del predatore. La velocità del predatore è
      // modificata con la funzione apply_rules_predator, successivamente è
      // limitata con velocity_limit ed in infine la posizione è aggiornata
      // con updatePosition
      if (Predator_on) {
        bds::apply_rules_predator(predator, flock, f, windowWidth,
                                  windowHeight);

        bds::velocity_limit(predator, Vmax * pred_coeff);
        assert(predator.absolute_velocity() <= Vmax * pred_coeff + 0.0001);

        predator.update_position();
        assert(predator.get_position().x <= windowWidth);
        assert(predator.get_position().y <= windowHeight);
        assert(predator.get_position().x >= 0);
        assert(predator.get_position().y >= 0);

        bds::eat(predator, flock, range);
      }

      // la finestra grafica viene colorata di bianco
      window.clear(sf::Color::White);

      // i boids vengono disegnati sulla finestra grafica
      std::for_each(flock.begin(), flock.end(), [&](bds::Boid& boid_i) {
        sf::CircleShape Boid_point(2);
        Boid_point.setFillColor(sf::Color::Black);
        Boid_point.setPosition(static_cast<float>(boid_i.get_position().x),
                               static_cast<float>(boid_i.get_position().y));
        window.draw(Boid_point);
      });

      // il predatore viene disegnato sulla finestra grafica
      if (Predator_on) {
        sf::CircleShape predator_point(range - 4);
        predator_point.setFillColor(sf::Color::Red);
        predator_point.setPosition(
            static_cast<float>(predator.get_position().x),
            static_cast<float>(predator.get_position().y));
        window.draw(predator_point);
      }

      // la scena è disegnata
      window.display();

      // loop della finetra di statistica
      if (window2.isOpen()) {
        bds::Statistics data = bds::stats(flock, d);
        window2.clear(sf::Color::White);
        sf::Text text;

        // Stampa delle statistiche sulla finestra
        text.setFont(font);
        text.setString("Distanza media: " + std::to_string(data.speed_mean)
                       + " cm/s " + "\n\n" + "Deviazione standard: "
                       + std::to_string(data.speed_err) + " cm/s " + "\n\n"
                       + "Velocità media: " + std::to_string(data.dist_mean)
                       + " cm " + "\n\n" + "Deviazione standard: "
                       + std::to_string(data.dist_err) + " cm ");
        text.setCharacterSize(7);
        text.setFillColor(sf::Color::Black);
        text.setPosition(5, 5);
        window2.draw(text);
        window2.display();
      }
    }
  } catch (const std::exception& ex) {
    std::cerr << "Errore: " << ex.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Errore imprevisto" << '\n';
    return EXIT_FAILURE;
  }
}
