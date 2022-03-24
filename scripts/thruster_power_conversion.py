THRUSTER_DATA_BIT_SIZE = 5

TLF_SHIFT_VAL = 0*THRUSTER_DATA_BIT_SIZE
TLT_SHIFT_VAL = 1*THRUSTER_DATA_BIT_SIZE
TLB_SHIFT_VAL = 2*THRUSTER_DATA_BIT_SIZE
TRF_SHIFT_VAL = 3*THRUSTER_DATA_BIT_SIZE
TRT_SHIFT_VAL = 4*THRUSTER_DATA_BIT_SIZE
TRB_SHIFT_VAL = 5*THRUSTER_DATA_BIT_SIZE

ENCODE_OFFSET = 2**(THRUSTER_DATA_BIT_SIZE-1)

def encode_msg(thruster_power_level):
    tlf_bits = (thruster_power_level["tlf"]+ENCODE_OFFSET) << TLF_SHIFT_VAL
    tlt_bits = (thruster_power_level["tlt"]+ENCODE_OFFSET) << TLT_SHIFT_VAL
    tlb_bits = (thruster_power_level["tlb"]+ENCODE_OFFSET) << TLB_SHIFT_VAL
    trf_bits = (thruster_power_level["trf"]+ENCODE_OFFSET) << TRF_SHIFT_VAL
    trt_bits = (thruster_power_level["trt"]+ENCODE_OFFSET) << TRT_SHIFT_VAL
    trb_bits = (thruster_power_level["trb"]+ENCODE_OFFSET) << TRB_SHIFT_VAL

    return (tlf_bits | tlt_bits | tlb_bits | trf_bits | trt_bits | trb_bits)