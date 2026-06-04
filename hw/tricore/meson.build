tricore_ss = ss.source_set()
tricore_ss.add(when: 'CONFIG_TRICORE_TESTBOARD', if_true: files('tricore_testboard.c'))
tricore_ss.add(when: 'CONFIG_TRICORE_TESTBOARD', if_true: files('tricore_testdevice.c'))
tricore_ss.add(when: 'CONFIG_TRIBOARD', if_true: files('triboard.c'))
tricore_ss.add(when: 'CONFIG_TC27X_SOC', if_true: files('tc27x_soc.c'))
tricore_ss.add(when: 'CONFIG_TC1797_SOC', if_true: files('tc1797_soc.c', 'pcp.c', 'tc1797_can.c', 'tc1797_adc.c', 'tc1797_gpta.c', 'tc1797_eray.c', 'tc1797_dma.c', 'tc1797_asc.c', 'tc1797_fadc.c', 'tc1797_mli.c', 'tc1797_jtag.c'))

hw_arch += {'tricore': tricore_ss}
