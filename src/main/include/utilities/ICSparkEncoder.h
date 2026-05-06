#pragma once

#include <memory>
#include <rev/RelativeEncoder.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>

class ICSparkEncoder {
 public:
  ICSparkEncoder(rev::spark::SparkRelativeEncoder& inbuilt);
  enum EncoderType { INBUILT, ABSOLUTE, RELATIVE };

  void SetPosition(double pos);
  void UseRelative(rev::spark::SparkMaxAlternateEncoder& encoder);
  void UseRelative(rev::spark::SparkFlexExternalEncoder& encoder);
  void UseAbsolute(rev::spark::SparkAbsoluteEncoder& encoder);

  double GetPosition();
  double GetVelocity();

 private:
  rev::spark::SparkRelativeEncoder _inbuilt;
  std::unique_ptr<rev::spark::SparkAbsoluteEncoder> _absolute;
  std::unique_ptr<rev::RelativeEncoder> _relative;
  double _absoluteSimPos = 0;
  EncoderType _selected = INBUILT;
};