
OBJS += techlibs/pango/synth_pango.o
OBJS += techlibs/pango/score.o
OBJS += techlibs/pango/stitcher_pango.o

$(eval $(call add_share_file,share/pango,techlibs/pango/pango_lib.v))
$(eval $(call add_share_file,share/pango,techlibs/pango/pango_sim.v))


