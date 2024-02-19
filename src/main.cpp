#include <list>
#include <numeric>
#include <SFML/Graphics.hpp>

namespace Helpers {
    float distance(sf::Vector2f a, sf::Vector2f b) {
        return sqrtf(pow(abs(b.x - a.x), 2) + pow(abs(b.y - a.y), 2));
    }

    float angle_to(sf::Vector2f a, sf::Vector2f b) {
        sf::Vector2f difference = b - a;
        const float angleRadians = atan2(difference.y, difference.x);
        const float angleDegrees = angleRadians * 180.0f / static_cast<float>(M_PI);

        return angleDegrees;
    }

    float normalize_angle(float theta) {
        // Normalize theta to the range [0, 360)
        theta = fmod(theta, 360);
        if (theta < 0) {
            theta += 360;
        }
        return theta;
    }

    float opposite_angle(float theta) {
        theta = normalize_angle(theta); // Ensure theta is within [0, 360)
        return normalize_angle(theta + 180); // Normalize again in case of edge cases
    }

    float steering_nudge(const float from, const float to, const float steering_force) {
        const float n_from = normalize_angle(from);
        const float n_to = normalize_angle(to);

        float difference = n_to - n_from;

        if (difference > 180.0f) {
            difference -= 360.0f;
        } else if (difference < -180.0f) {
            difference += 360.0f;
        }

        // Apply steering force but do not exceed the difference in either direction
        const float adjustment = (fabs(difference) < steering_force) ? fabs(difference) : steering_force;
        const float newAngle = n_from + ((difference > 0) ? adjustment : -adjustment);

        // Normalize the new angle to ensure it's within [0, 360)
        return normalize_angle(newAngle);
    }

    float degrees_to_radians(const float degrees) {
        return degrees * (3.141592653f / 180);
    }
}

struct Boid {
public:
    sf::Vector2f pos;
    float rot;
    size_t id;

    Boid() = delete;
    Boid(sf::Vector2f _pos, float _rot) : pos(_pos), rot(_rot) {
        static size_t _id = 0;
        id = _id++;
    }

    bool operator==(const Boid& other) const {
        return id == other.id;
    }
};

struct System {
    std::list<Boid> boids;

    explicit System(size_t n_boids) {
        constexpr float spacing_x = 20;
        for(size_t i = 0; i < n_boids; ++i) {
            boids.emplace_back(sf::Vector2f(spacing_x * static_cast<float>(i), 200), 0);
        }
    }

    void update(float delta_time) {
        constexpr float separation_force = 90;
        constexpr float cohesion_force = 60;
        constexpr float alignment_force = 50;
        constexpr float movement_speed = 10;

        // Separation
        // For all boids in circle, find the average heading, and invert that. Steer towards that heading.
        constexpr float separationRing = 20;
        for(auto& x : boids) {
            std::vector<float> boid_angles;
            for(auto& y : boids) {
                if(x == y) continue;

                if(Helpers::distance(x.pos, y.pos) < separationRing) {
                    boid_angles.push_back(Helpers::angle_to(x.pos, y.pos));
                }
            }

            if(boid_angles.empty()) continue;
            const float average_angle = std::accumulate(boid_angles.begin(), boid_angles.end(), 0.0f) / static_cast<float>(boid_angles.size());
            const float target_angle = Helpers::opposite_angle(average_angle);

            x.rot = Helpers::steering_nudge(x.rot, target_angle, separation_force * delta_time);
        }

        // Cohesion
        // Find the center of mass of all boids. Steer towards it.
        std::vector<float> xs;
        std::vector<float> ys;
        for(auto& boid : boids) {
            xs.push_back(boid.pos.x);
            ys.push_back(boid.pos.y);
        }
        float center_x = std::accumulate(xs.begin(), xs.end(), 0.0f) / static_cast<float>(xs.size());
        float center_y = std::accumulate(ys.begin(), ys.end(), 0.0f) / static_cast<float>(ys.size());

        for(auto& boid : boids) {
            boid.rot = Helpers::steering_nudge(boid.rot, Helpers::angle_to(boid.pos, {center_x, center_y}), cohesion_force * delta_time);
        }

        // Alignment
        // Find the average heading of all boids. Steer towards it.
        std::vector<float> rots;

        for(auto& boid : boids) {
            rots.push_back(boid.rot);
        }
        float average_rotation = std::accumulate(rots.begin(), rots.end(), 0.0f) / static_cast<float>(rots.size());

        for(auto& boid : boids) {
            boid.rot = Helpers::steering_nudge(boid.rot, average_rotation, alignment_force * delta_time);
        }

        // Movement
        // Move with respect to the rotation

        for(auto& boid : boids) {
            boid.pos += sf::Vector2f(
                std::cos(Helpers::degrees_to_radians(boid.rot)) * movement_speed * delta_time,
                std::sin(Helpers::degrees_to_radians(boid.rot)) * movement_speed * delta_time
            );
        }
    }

    void render(sf::RenderWindow& window) const {
        sf::RectangleShape rs;
        rs.setSize({1, 4});
        rs.setFillColor(sf::Color::White);
        for(auto& boid : boids) {
            rs.setPosition(boid.pos);
            rs.setRotation(boid.rot);
            window.draw(rs);
        }
    }

    [[nodiscard]] sf::Vector2f average_boid_position() const {
        std::vector<float> x_pos;
        std::vector<float> y_pos;

        for(auto& boid : boids) {
            x_pos.push_back(boid.pos.x);
            y_pos.push_back(boid.pos.y);
        }

        const float x = std::accumulate(x_pos.begin(), x_pos.end(), 0.0f);
        const float y = std::accumulate(y_pos.begin(), y_pos.end(), 0.0f);

        return {x / static_cast<float>(x_pos.size()), y / static_cast<float>(y_pos.size())};
    }
};

int main() {
    sf::RenderWindow window(sf::VideoMode(800, 600), "boids");
    window.setFramerateLimit(60);
    sf::View view;

    System system(20);

    sf::Clock clock;

    constexpr size_t updates_per_tick = 3;

    while(window.isOpen()) {
        float delta_time = clock.restart().asSeconds();

        sf::Event event{};
        while(window.pollEvent(event)) {
            if(event.type == sf::Event::Closed) {
                window.close();
            }
        }
        window.clear();

        for(int i = 0; i < updates_per_tick; ++i) {
            system.update(delta_time);
        }

        view = sf::View(system.average_boid_position(), sf::Vector2f(800, 200));
        view.zoom(0.5);
        window.setView(view);

        system.render(window);

        window.display();
    }
    return 0;
}