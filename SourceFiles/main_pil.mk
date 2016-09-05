#
#	Makefile for Exp3C6713
#

PEOS_PATH=$(MWINSTALL)/PEOS/$(TARGET_TYPE)/$(PEOS_VER)

DIR_1= .

DIR_2= $(MWPATH)/cgtools/include

DIR_3= $(PEOS_PATH)/lib

#DIR_4= $(MWPATH)/bios/include

DIR_5= $(MWPATH)/dsk6713/include

DIR_6= $(MWPATH)/cgtools/bin

DIR_7= $(DIR_3)

#DIR_8= $(MWPATH)/dsk6713/lib

CC=	"$(DIR_6)/cl6x"

CFLAGS=	-g -k -q -al -fr"$(DIR_1)" -i"$(DIR_3)" -i"$(DIR_5)" -i"$(DIR_4)" -d"CHIP_6713" -mv6710 -ml1

CPPFLAGS=	$(CFLAGS) -ppo -dcregister= -dfar= -dinterrupt=

CNV=hex6x

include	$(TARGET_NAME).sub

clean:
	@rm -f $(TARGET_NAME).def $(TARGET_NAME).mot $(TARGET_NAME).asm $(TARGET_NAME).map $(OBJS)

rebuild: clean all

LOADMODULE:	$(TARGET_NAME).mot $(TARGET_NAME).cnd
	@echo	Compile OK.

all:	CONFIG
all2:	LOADMODULE DEF CND

CONFIG:
	@case `$(CC) -version | head -1 | awk '{ print $$4 }' | tr '\015' ' ' | sed 's/ //g'` in	\
	Version)	\
		DIR_4='$(MWPATH)/bios/include';	\
		DIR_8='$(MWPATH)/dsk6713/lib';	\
		export	DIR_4 DIR_8;			\
		$(MAKE) -e -f $(TARGET_NAME).mk all2;;				\
	v5.0.0)		\
		DIR_4='$(MWPATH)/bios/include';	\
		DIR_8='$(MWPATH)/dsk6713/lib';	\
		export	DIR_4 DIR_8;			\
		$(MAKE) -e -f $(TARGET_NAME).mk all2;;				\
	v5.1.0)		\
		DIR_4='$(MWPATH)/csl/include';	\
		DIR_8='$(MWPATH)/csl/lib';		\
		export	DIR_4 DIR_8;			\
		$(MAKE) -e -f $(TARGET_NAME).mk all2;;				\
	esac

$(TARGET_NAME).fbs: 
	@LENGTH=0x28000; export LENGTH; macexp $(PEOS_PATH)/config/orgflash.cmd flash.cmd
	@$(CNV) flash.cmd
	@LENGTH=`length 7f00 < $(TARGET_NAME).mot`; export LENGTH; macexp $(PEOS_PATH)/config/orgflash.cmd flash.cmd
	@$(CNV) flash.cmd
	@grep -v S7 $(TARGET_NAME).mot | wcnv 1 50 | sort > $(TARGET_NAME).new
	@grep 'ENTRY POINT SYMBOL:' $(TARGET_NAME).map | awk '{ print $$6 }' | mssum 7 >> $(TARGET_NAME).new
	@mv $(TARGET_NAME).new $(TARGET_NAME).mot
	@rm -f $(TARGET_NAME).asm $(TARGET_NAME).lst $(TARGET_NAME).obj $(TARGET_NAME).out flash.cmd

$(DIR_1)/$(TARGET_NAME).obj: patch

$(TARGET_NAME).map:	$(TARGET_NAME).mot

$(DIR_1)/$(TARGET_NAME).mot: $(FRC) $(EXOBJS) $(OBJS) $(EXTLIBS)
	-@echo -z -q -c -m"$(TARGET_NAME).map" -o"$(DIR_1)/$(TARGET_NAME).out" -x -i"$(DIR_7)" -i"$(DIR_8)" -l"csl6713.lib" -l"rts6700.lib" -l"dsk6713bsl.lib" > $(TARGET_NAME).lkf
	-@echo "$(EXOBJS)" >> $(TARGET_NAME).lkf
	-@echo "$(OBJS)" >> $(TARGET_NAME).lkf
	-@echo "$(EXTLIBS)" >> $(TARGET_NAME).lkf
	-@echo "$(DIR_7)/mwio3.lib" >> $(TARGET_NAME).lkf
	-@echo "$(TARGET_NAME).cmd" >> $(TARGET_NAME).lkf
	@$(CC) -@"$(TARGET_NAME).lkf"
	-@rm -f $(TARGET_NAME).lkf
	@$(MAKE) $(MFLAGS) -f $(TARGET_NAME).mk $(TARGET_NAME).fbs

DEF:	$(TARGET_NAME).def
	@echo	Symbol Information OK.

#
#	for Symbol Infor.
#
$(TARGET_NAME).var:	$(TARGET_NAME).pp
	@-cc1 $(TARGET_NAME).pp > $(TARGET_NAME).tmp 2> $(TARGET_NAME).err
	@sed 's/^/_/' $(TARGET_NAME).tmp | sort | uniq |\
	sed -f $(MWINSTALL)/bin/MW_COMPILE/type.sed | d2u > $(TARGET_NAME).typ
	@rm $(TARGET_NAME).pp $(TARGET_NAME).tmp $(TARGET_NAME).err $(TARGET_NAME).s
	@awk '{ print $$1 }' < $(TARGET_NAME).typ > $(TARGET_NAME).var

$(TARGET_NAME).def:	$(TARGET_NAME).map $(TARGET_NAME).var
	@map2symTI 1 < $(TARGET_NAME).map | tr '\015' ' ' | grep -w -f $(TARGET_NAME).var |\
	awk '{ print $$2, $$1, 2}' |\
	perl $(MWINSTALL)/bin/MW_COMPILE/Type.pl $(TARGET_NAME).typ $(PEOS_PATH)/config/Type.cfg |\
	awk '{ if ( $$4 ~ /[0-9]$$*/ ) print }'| d2u > $(TARGET_NAME).def
	@rm $(TARGET_NAME).var $(TARGET_NAME).typ

patch:
	@rm -f $(DIR_3)/csl_timer.h
	@touch $(DIR_3)/csl_timer.h
	@cp $(DIR_4)/csl_timer.h $(DIR_3)
	@chmod +w $(DIR_3)/csl_timer.h
	@patch $(DIR_3)/csl_timer.h $(DIR_3)/csl_timer.pat

CND:	$(TARGET_NAME).cnd

$(TARGET_NAME).cnd:
	@echo $(TARGET_NAME).c > $@
	@cat $(MWINSTALL)/bin/MW_VIO/default.cnd >> $@

.SUFFIXES:	.obj .asm .c .h .lib .pp .C

.c.obj:
	@$(CC) $(CFLAGS) $<

.c.pp:
	@$(CC) $(CPPFLAGS) $<

.C.obj:
	@$(CC) $(CFLAGS) $<

.C.pp:
	@$(CC) $(CPPFLAGS) $<
