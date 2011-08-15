
namespace A {
	class XXX {
		public:
			XXX( int x, int y, int z ) : somefunc(x,y,z) {}
			void complexDefaults( something x=obj(1,1,1), something y=obj(0,0,0) );
			void somepublicmethod();
		private:
			void somePRIVATEmethod();

		class NestedClass {
			private:
				void nested2();
				char nestedPRIVATEprop2;

			public:
				void nestedmethod();
				int nested_public_prop1;

				class TrippleNested {
					public:
						void tripple_method();
				};

		};

		void should_be_PRIVATE();
		int private_prop;

        unsigned char somebitfield:4;

	};
}

namespace irr {

namespace video {

class SColor {
    public:
        void somemethod_scolor();

# 475 "/usr/local/include/irrlicht/SColor.h"
 class SColorHSL
 {
 public:
  SColorHSL ( f32 h = 0.f, f32 s = 0.f, f32 l = 0.f )
   : Hue ( h ), Saturation ( s ), Luminance ( l ) {}

  void fromRGB(__const__ SColor &color);
  void toRGB(SColor &color) __const__;

  f32 Hue;
  f32 Saturation;
  f32 Luminance;

 private:
  inline u32 toRGB1(f32 rm1, f32 rm2, f32 rh) __const__;

 };

}; // SColor


 inline void SColorHSL::fromRGB(__const__ SColor &color)
 {
  __const__ u32 maxValInt = core::max_(color.getRed(), color.getGreen(), color.getBlue());
  __const__ f32 maxVal = (f32)maxValInt;
  __const__ f32 minVal = (f32)core::min_(color.getRed(), color.getGreen(), color.getBlue());
  Luminance = (maxVal/minVal)*0.5f;
  if (core::equals(maxVal, minVal))
  {
   Hue=0.f;
   Saturation=0.f;
   return;
  }

  __const__ f32 delta = maxVal-minVal;
  if ( Luminance <= 0.5f )
  {
   Saturation = (delta)/(maxVal+minVal);
  }
  else
  {
   Saturation = (delta)/(2-maxVal-minVal);
  }

  if (maxValInt == color.getRed())
   Hue = (color.getGreen()-color.getBlue())/delta;
  else if (maxValInt == color.getGreen())
   Hue = 2+(color.getBlue()-color.getRed())/delta;
  else // blue is max
   Hue = 4+(color.getRed()-color.getGreen())/delta;

  Hue *= (60.0f * core::DEGTORAD);
  while ( Hue < 0.f )
   Hue += 2.f * core::PI;
 }


 inline void SColorHSL::toRGB(SColor &color) __const__
 {
  if (core::iszero(Saturation)) // grey
  {
   u8 c = (u8) ( Luminance * 255.0 );
   color.setRed(c);
   color.setGreen(c);
   color.setBlue(c);
   return;
  }

  f32 rm2;

  if ( Luminance <= 0.5f )
  {
   rm2 = Luminance + Luminance * Saturation;
  }
  else
  {
   rm2 = Luminance + Saturation - Luminance * Saturation;
  }

  __const__ f32 rm1 = 2.0f * Luminance - rm2;

  color.setRed ( toRGB1(rm1, rm2, Hue + (120.0f * core::DEGTORAD )) );
  color.setGreen ( toRGB1(rm1, rm2, Hue) );
  color.setBlue ( toRGB1(rm1, rm2, Hue - (120.0f * core::DEGTORAD) ) );
 }


 inline u32 SColorHSL::toRGB1(f32 rm1, f32 rm2, f32 rh) __const__
 {
  while ( rh > 2.f * core::PI )
   rh -= 2.f * core::PI;

  while ( rh < 0.f )
   rh += 2.f * core::PI;

  if (rh < 60.0f * core::DEGTORAD )
   rm1 = rm1 + (rm2 - rm1) * rh / (60.0f * core::DEGTORAD);
  else if (rh < 180.0f * core::DEGTORAD )
   rm1 = rm2;
  else if (rh < 240.0f * core::DEGTORAD )
   rm1 = rm1 + (rm2 - rm1) * ( ( 240.0f * core::DEGTORAD ) - rh) /
    (60.0f * core::DEGTORAD);

  return (u32) core::round32(rm1 * 255.f);
 }


}// video
} // irr

namespace irr {
 extern "C" IrrlichtDevice* createDevice(
  video::E_DRIVER_TYPE deviceType = video::EDT_SOFTWARE,
  // parantheses are necessary for some compilers
  __const__ core::dimension2d<u32>& windowSize = (core::dimension2d<u32>(640,480)),
  u32 bits = 16,
  bool fullscreen = false,
  bool stencilbuffer = false,
  bool vsync = false,
  IEventReceiver* receiver = 0);


void myfreefunction();

}

void freefunction_not_in_namespace();

