#include "operator.hpp"
#include <SFML/Graphics.hpp>
#include <iostream>
#include <random>

int main()
{
  try {
    // parametri in input
    bool cin_on{true};
    int n{100};
    double d{90};
    double ds{15};
    double s{0.5};
    double a{0.1};
    double c{0.05};
    bool Predator_on{false};
    double e = (Predator_on) ? 2 : 0;
    double f{2.5};
    double const Vmax{10 / bds::conv_fac}; // non cambiare
    double const range{8};                 //[NON MODIFICARE]
    double pred_coeff{1.3};

    // Lettura dei parametri con cin
    if (cin_on) {
      std::cout << "Scegliere la modalità con (1) o senza predatore (0): \n";
      std::cin >> Predator_on;
      if (Predator_on == true) {
        std::cout
            << "Inserire in ordine : numero di boids , d , ds , s , a , c , "
               "e , f , pred_coeff\n";
        std::cin >> n >> d >> ds >> s >> a >> c >> e >> f >> pred_coeff;
      } else {
        std::cout
            << "Inserire in ordine : numero di boids , d , ds , s , a , c \n";
        std::cin >> n >> d >> ds >> s >> a >> c;
      }
      if (std::cin.fail()) {
        throw std::invalid_argument(
            "Input non valido. Si prega di inserire i valori corretti.");
      }
    }

    // selezione del font per la scrittura delle statistiche dei boids
    sf::Font font;
    font.loadFromFile("./Nexa-Heavy.ttf");

    if (!font.loadFromFile("./Nexa-Heavy.ttf")) {
      throw std::runtime_error("Impossibile caricare il font.");
    }

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

    // generazione delle variabili per lo spawn dei boids
    std::random_device r;
    std::default_random_engine eng(r());
    std::vector<bds::Boid> flock;
    std::uniform_real_distribution<> roll_diceX(20, windowWidth - 20);
    std::uniform_real_distribution<> roll_diceY(20, windowHeight - 20);
    std::uniform_real_distribution<> roll_diceVx(-Vmax / 2, Vmax / 2);
    std::uniform_real_distribution<> roll_diceVy(-Vmax / 2, Vmax / 2);

    // loop di generazione dei boids
    for (int i = 0; i < n; i++) {
      bds::Boid boid_i{roll_diceX(eng), roll_diceY(eng), roll_diceVx(eng),
                       roll_diceVy(eng)};

      assert(boid_i.position().x <= windowWidth);
      assert(boid_i.position().y <= windowHeight);
      assert(boid_i.position().x >= 0);
      assert(boid_i.position().y >= 0);

      assert(boid_i.absoluteVelocity() <= Vmax);

      flock.push_back(boid_i);
    }

    // generazione del predatoratore
    bds::Boid predator{roll_diceX(eng), roll_diceY(eng), roll_diceVx(eng),
                       roll_diceVy(eng)};

    assert(predator.position().x <= windowWidth);
    assert(predator.position().y <= windowHeight);
    assert(predator.position().x >= 0);
    assert(predator.position().y >= 0);

    assert(predator.absoluteVelocity() <= Vmax * pred_coeff);

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
      // modificata con la funzione applyRules, successivamente è limitata con
      // velocitylimit ed in infine la posizione è aggiornata con updatePosition
      for (bds::Boid& boid_i : flock) {
        bds::applyRules(boid_i, a, c, s, d, ds, e, windowWidth, windowHeight,
                        flock, predator);

        bds::velocityLimit(boid_i, Vmax);
        assert(boid_i.absoluteVelocity() <= Vmax + 0.0001);

        boid_i.updatePosition();
        assert(boid_i.position().x <= windowWidth);
        assert(boid_i.position().y <= windowHeight);
        assert(boid_i.position().x >= 0);
        assert(boid_i.position().y >= 0);
      }

      // cambio di posizione del predatore. La velocità del predatore è
      // modificata con la funzione RulesPred, successivamente è limitata con
      // velocitylimit ed in infine la posizione è aggiornata con updatePosition
      if (Predator_on) {
        bds::rulesPred(predator, flock, f, windowWidth, windowHeight);

        bds::velocityLimit(predator, Vmax * pred_coeff);
        assert(predator.absoluteVelocity() <= Vmax * pred_coeff + 0.0001);

        predator.updatePosition();
        assert(predator.position().x <= windowWidth);
        assert(predator.position().y <= windowHeight);
        assert(predator.position().x >= 0);
        assert(predator.position().y >= 0);

        bds::eat(predator, flock, range);
      }

      // la finestra grafica viene colorata di bianco
      window.clear(sf::Color::White);

      // i boids vengono disegnati sulla finestra grafica
      for (bds::Boid& b : flock) {
        sf::CircleShape Boid_point(2);
        Boid_point.setFillColor(sf::Color::Black);
        Boid_point.setPosition(static_cast<float>(b.position().x),
                               static_cast<float>(b.position().y));
        window.draw(Boid_point);
      }

      // il predatore viene disegnato sulla finestra grafica
      if (Predator_on) {
        sf::CircleShape predator_point(range - 4);
        predator_point.setFillColor(sf::Color::Red);
        predator_point.setPosition(static_cast<float>(predator.position().x),
                                   static_cast<float>(predator.position().y));
        window.draw(predator_point);
      }

      // la scena è disegnata
      window.display();

      // loop della finetra di statistica
      if (window2.isOpen()) {
        bds::Statistics data = bds::stats(flock);
        window2.clear(sf::Color::White);
        sf::Text text;

        // Stampa delle statistiche sulla finestra
        text.setFont(font);
        text.setString("Avarage velocity: " + std::to_string(data.speed_mean)
                       + " cm/s " + "\n\n" + "Standard deviation: "
                       + std::to_string(data.speed_err) + " cm/s " + "\n\n"
                       + "Avarage distance: " + std::to_string(data.dist_mean)
                       + " cm " + "\n\n" + "Standard deviation: "
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
