#include "operator.hpp"
#include <SFML/Graphics.hpp>
#include <iostream>
#include <random>

int main()
{
  bool cin_on{false};  //modalità con gestione dei parametri tramite cin
  int n{0};          //numero di boid nella separazione
  double d{900};        //distanza di visione del boid
  double ds{15};       //distanza di separazione
  double s{0.5};       //fattore di moltiplicazione per la regola di separazione
  double a{0.1};       //fattore di moltiplicazione per la regola di allineamento
  double c{0.05};      //fattore di moltiplicazione per la regola di coesione
  bool Predator_on{true};           //modalità con predatore
  double e = (Predator_on) ? 2 : 0; //fattore di moltiplicazione per la regola di fuga
  double f{2.5};                    //fattore di moltiplicazione per la regola di inseguimento  
  double Vmax{6 / 0.02236842105};    //velocità massima
  const double range{8};            //[NON MODIFICARE] distanza con cui i boids vengono mangiati dal predatore
  const double pred_coeff{1.3};     //percentuale della velocità del predatore rispetto alla vellocità massima

  //Lettura dei parametri con cin
  if (cin_on) {
    std::cout << "Scegliere la modalità con (1) o senza predatore (0): \n";
    std::cin >> Predator_on;
    if (Predator_on == true) {
      std::cout
          << "Inserire in ordine : numero di boids , d , ds , s , a , c , "
             "e , f \n";
      std::cin >> n >> d >> ds >> s >> a >> c >> e >> f;
    } else {
      std::cout
          << "Inserire in ordine : numero di boids , d , ds , s , a , c \n";
      std::cin >> n >> d >> ds >> s >> a >> c;
    }
  }

  //selezione del font per la scrittura delle statistiche dei boids
  sf::Font font;
  font.loadFromFile("./Nexa-Heavy.ttf");

  //creazione della finestra di sfml grafica 
  unsigned int windowWidth  = (1) * sf::VideoMode::getDesktopMode().width - 40;
  unsigned int windowHeight = (1) * sf::VideoMode::getDesktopMode().height - 75;
  sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight),
                          "Boids Simulation");
  window.setFramerateLimit(30);
  window.setPosition(sf::Vector2i(0, 0));

  //creazione della finestra delle statistiche
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

  //loop di generazione dei boids
  for (int i = 0; i < n; i++) {
    bds::Boid boid_i{roll_diceX(eng), roll_diceY(eng), roll_diceVx(eng),
                     roll_diceVy(eng)}; // implicita conv double int
    flock.push_back(boid_i);
  }

  //generazione del predatore
  bds::Boid pred{roll_diceX(eng), roll_diceY(eng), roll_diceVx(eng),
                 roll_diceVy(eng)};

  bds::Boid bpoiu{500,500,0,0};

  //inizio gameloop di sfml
  while (window.isOpen()
         | window2.isOpen()) {
    
    //chiusura delle finestre
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

    //loop di cambio di posizione dei boids. La velocità dei boids viene modificata con la funzione 
    //applyRules, successivamente è limitata con velocitylimit ed in infine la posizione è 
    //aggiornata con updatePosition 
    for (bds::Boid& b1 : flock) {
      bds::applyRules(b1, a, c, s, d, ds, e, windowWidth, windowHeight, flock,
                      pred);
      bds::velocitylimit(b1, Vmax);
      b1.updatePosition();
      assert(b1.position().x <= windowWidth + 100);
      assert(b1.position().y <= windowHeight + 100);
      assert(b1.position().x >= -100);
      assert(b1.position().y >= -100);
    }

    //cambio di posizione del predatore. La velocità del predatore è modificata con la funzione 
    //RulesPred, successivamente è limitata con velocitylimit ed in infine la posizione è 
    //aggiornata con updatePosition 
    if (Predator_on) {
      bds::RulesPred(pred, flock, f, windowWidth, windowHeight);
      bds::velocitylimit(pred, Vmax * pred_coeff);
      pred.updatePosition();
      bds::eat(pred, flock, range);
    }

    //la finestra grafica viene colorata di bianco
    window.clear(sf::Color::White);

    //i boids vengono disegnati sulla finestra grafica
    for (bds::Boid& b : flock) { 
      sf::CircleShape Boid_point(2);
      Boid_point.setFillColor(sf::Color::Black);
      Boid_point.setPosition(
          static_cast<float>(b.position().x),
          static_cast<float>(
              b.position().y)); 
      window.draw(Boid_point);
    }

    //il predatore viene disegnato sulla finestra grafica
    if (Predator_on) {
      sf::CircleShape pred_point(4);
      pred_point.setFillColor(sf::Color::Red);
      pred_point.setPosition(static_cast<float>(bpoiu.position().x),
                             static_cast<float>(bpoiu.position().y));
      window.draw(pred_point);
    }

    //la scena è disegnata
    window.display();

    //loop della finetra di statistica
    if (window2.isOpen()) {
      bds::Statistics data = bds::stats(flock);
      window2.clear(sf::Color::White);
      sf::Text text;

      //Stampa delle statistiche sulla finestra
      text.setFont(font);
      text.setString("Avarage velocity: " + std::to_string(data.speed_mean)
                     + " cm/s " + "\n\n" + "Standard deviation: "
                     + std::to_string(data.speed_err) + " cm/s " + "\n\n"
                     + "Avarage distance: " + std::to_string(data.dis_mean)
                     + " cm " + "\n\n" + "Standard deviation: "
                     + std::to_string(data.dis_err) + " cm ");
      text.setCharacterSize(7);
      text.setFillColor(sf::Color::Black);
      text.setPosition(5, 5);
      window2.draw(text);
      window2.display();
    }
  }
}
