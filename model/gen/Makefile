ASN1_DEFS = ITS_CAM_v1.3.2.asn

include Makefile.am.sample
TARGET=cam_parser
ASN1_OBJS=${ASN_MODULE_SOURCES:.c=.o}

all: cam_size

cam_size.o: cam_size.c
	clang -I. -c cam_size.c

Makefile.am.sample: $(ASN1_DEFS)
	asn1c -gen-PER -pdu=CAM $(ASN1_DEFS)

cam_size: cam_size.o Makefile.am.Sample
	clang -g -o cam_size cam_size.o $(ASN1_OBJS)

cam_parser: Makefile.am.Sample
	make -f Makefile.am.Sample TARGET=cam_parser

.PHONY: parse_sample
parse_sample: cam_parser
	./cam_parser -iper ./sample_cam.uper

.PHONY: parse_sample_xml
parse_sample_xml: cam_parser
	./cam_parser -o per -ixer ./sample_cam.xml | xxd
