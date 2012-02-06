#if defined(__linux__) && defined(__GNUC__)
#define _GNU_SOURCE
#include <fenv.h>
#endif
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include "MEM_guardedalloc.h"
#ifdef WIN32
#include "BLI_winstuff.h"
#endif

#include "BLF_api.h"

#include "BLI_args.h"
#include "BLI_threads.h"
#include "BLI_scanfill.h"
#include "BLI_utildefines.h"
#include "BLI_blenlib.h"
#include "BLI_ghash.h"
#include "BLI_blenlib.h"

#include "BLI_math.h"
#include "BLI_voxel.h"


// keep in order
#include "DNA_ID.h"
#include "DNA_scene_types.h"
// must come after
#include "BKE_blender.h"
#include "BKE_context.h"
#include "BKE_font.h"
#include "BKE_global.h"
#include "BKE_main.h"
#include "BKE_material.h"
#include "BKE_packedFile.h"
#include "BKE_scene.h"
#include "BKE_node.h"
#include "BKE_report.h"
#include "BKE_sound.h"
#include "BKE_depsgraph.h"
#include "BKE_utildefines.h"

#include "BKE_idprop.h"
#include "BKE_library.h"
#include "BKE_screen.h"

#include "BKE_idcode.h"	// for BKE_idcode_from_name

#include "BKE_image.h"

#include "BKE_DerivedMesh.h"
#include "BKE_collision.h"
#include "BKE_cloth.h"		// contains implicit_solver
#include "BKE_tracking.h"

#include "IMB_imbuf_types.h"	// for struct ImBuf def
#include "IMB_imbuf.h"		// for IMB_init

#include "BPY_extern.h"

#include "RE_pipeline.h"
#include "RE_shader_ext.h"	// for RE_bake_shade_all_selected, RE_bake_shade_get_image
#include "RE_engine.h"

#include "ED_datafiles.h"
#include "ED_armature.h"
#include "ED_keyframing.h"
#include "ED_node.h"
#include "ED_render.h"
#include "ED_space_api.h"
#include "ED_screen.h"
#include "ED_util.h"
#include "ED_image.h"		// CTX_wm_space_image(C) returns SpaceImage
#include "ED_uvedit.h"		// ED_uvedit_assign_image

#include "WM_api.h"
#include "WM_types.h"
#include "wm.h"
#include "wm_event_system.h"
#include "wm_draw.h"
#include "wm_window.h"
#include "wm_subwindow.h"

#include "GPU_draw.h"
#include "GPU_extensions.h"
#include "GPU_buffers.h"


#include "BLO_readfile.h"
#include "BLO_writefile.h"

#include "BIK_api.h"

#include "GHOST_C-api.h"
#include "GHOST_Path-api.h"

#include "Bullet-C-Api.h"
#include "BKE_bullet.h"

#include "UI_interface.h"

#include "collada.h"

/* created by makesdna/rna */
//#include "RNA_blender.h"
//#include "RNA_define.h"

