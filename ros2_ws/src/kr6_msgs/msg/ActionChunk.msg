# One prediction from the policy: chunk_len actions of dim floats each,
# row-major in data. obs_seq ties the chunk to the observation it answered.
uint64 obs_seq
uint32 chunk_len
uint32 dim
float64[] data
