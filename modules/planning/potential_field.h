namespace apollo {
namespace planning {

// attractive potential gain
const float KP = 5.0;
// repulsive potential gain
const float ETA = 100.0;

class PotentialField {
 public:
  PotentialField();
  void calcPotentialField();
  float calcAttractivePotential();
  float calcRepulsivePotential();
  void Plan();

 private:
};
}  // namespace planning
}  // namespace apollo
