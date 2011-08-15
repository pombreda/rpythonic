namespace irr
{
namespace gui
{
 class IGUIFont;
 class IGUISpriteBank;
 class IGUIElement;

 //! Enumeration of available default skins.
 /** To set one of the skins, use the following code, for example to set

	the Windows classic skin:

	\code

	gui::IGUISkin* newskin = environment->createSkin(gui::EGST_WINDOWS_CLASSIC);

	environment->setSkin(newskin);

	newskin->drop();

	\endcode

	*/
# 30 "/usr/local/include/irrlicht/IGUISkin.h"
 enum EGUI_SKIN_TYPE
 {
  //! Default windows look and feel
  EGST_WINDOWS_CLASSIC=0,
  //! Like EGST_WINDOWS_CLASSIC, but with metallic shaded windows and buttons
  EGST_WINDOWS_METALLIC,
  //! Burning's skin
  EGST_BURNING_SKIN,

  //! An unknown skin, not serializable at present
  EGST_UNKNOWN,

  //! this value is not used, it only specifies the number of skin types
  EGST_COUNT
 };

