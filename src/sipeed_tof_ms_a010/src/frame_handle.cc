#include <string.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include "frame_struct.h"

#define DEBUG_FRAME(line)
// #define DEBUG_FRAME(line) line

frame_t *handle_process(std::string s) {
  static std::vector<uint8_t> vecChar;
  static const uint8_t sflag_l = FRAME_BEGIN_FLAG & 0xff;
  static const uint8_t sflag_h = (FRAME_BEGIN_FLAG >> 8) & 0xff;
  static const uint8_t eflag = FRAME_END_FLAG & 0xff;

  uint32_t frame_payload_len = 0;
  frame_t *pf = NULL;
  std::vector<uint8_t>::iterator it;

  // Declare variables here to avoid "crosses initialization" errors with goto later
  size_t total_size = 0;
  size_t current_frame_end_offset = 0;
  size_t remaining_start_index = 0;

  vecChar.insert(vecChar.end(), s.cbegin(), s.cend());

  if (vecChar.size() < 2) {
    DEBUG_FRAME(std::cerr << "data is not enough!" << std::endl);
    goto __finished;
  }

__find_header:
  it = vecChar.begin();
  do {
    /* find sflag_h from [1:] first and next in [it+1:] */
    it = find(it + 1, vecChar.end(), sflag_h);
    /* sflag_h not found */
    if (it == vecChar.end()) {
      /* keep last element which may be sflag_l */
      std::vector<uint8_t>(vecChar.end() - 1, vecChar.end()).swap(vecChar);
      DEBUG_FRAME(std::cerr << "frame head not found! wait more data." << std::endl);
      goto __finished;
    }
    /* sflag_h found, *(it-1) always valid */
  } while (*(it - 1) != sflag_l);
  /* we got *it==sflag_h and *(it-1)==sflag_l */

  if (it - 1 != vecChar.begin()) {
    std::vector<uint8_t>(it - 1, vecChar.end()).swap(vecChar);
    DEBUG_FRAME(std::cerr << "frame move to first!" << std::endl);
  }

  if (vecChar.size() < sizeof(frame_t)) {
    DEBUG_FRAME(std::cerr << "frame head data is not enough now! wait more data." << std::endl);
    goto __finished;
  }

  pf = (frame_t *)&vecChar[0];
  frame_payload_len = pf->frame_head.frame_data_len - FRAME_HEAD_DATA_SIZE;

  /* max frame payload size */
  if (frame_payload_len > 100 * 100) {
    DEBUG_FRAME(std::cerr << "frame payload length too large: " << frame_payload_len << std::endl);
    vecChar.erase(vecChar.begin()); // drop first byte so we don't try to parse same bad data again
    goto __find_header;
  }

  if (vecChar.begin() + FRAME_HEAD_SIZE + frame_payload_len +
          FRAME_CHECKSUM_SIZE + FRAME_END_SIZE + 1 >
      vecChar.end()) {
    DEBUG_FRAME(std::cerr << "expected frame payload length " << frame_payload_len
                  << " but buffer size is " << vecChar.size() << std::endl);
    DEBUG_FRAME(std::cerr << "frame payload data is not enough now! wait more data." << std::endl);
    goto __finished;
  }

  {
    static uint8_t check_sum = 0;
    check_sum = 0;
    for (uint32_t i = 0; i < FRAME_HEAD_SIZE + frame_payload_len; i++) {
      check_sum += ((uint8_t *)pf)[i];
    }
    if (check_sum != ((uint8_t *)pf)[FRAME_HEAD_SIZE + frame_payload_len] ||
        eflag != ((uint8_t *)pf)[FRAME_HEAD_SIZE + frame_payload_len +
                                 FRAME_CHECKSUM_SIZE]) {
      DEBUG_FRAME(std::cerr << "frame checksum or tail invalid! one more time." << std::endl);
      std::vector<uint8_t>(it, vecChar.end()).swap(vecChar);
      goto __find_header;
    }
  }

  // --- Safety Block Start (Fixed for Compilation & Memory Safety) ---

  total_size = sizeof(frame_t) + frame_payload_len;

  if (total_size < sizeof(frame_t)) {
      std::cerr << "[ERROR] Calculated total size underflow." << std::endl;
      return NULL;
  }

  // Safety Check: Verify malloc success immediately to avoid segfault in memcpy
  pf = (frame_t *)malloc(total_size);
  if (!pf) {
    std::cerr << "[CRITICAL] frame_handle.cc: Memory allocation failed for size "
              << total_size << ". Returning NULL." << std::endl;
    vecChar.clear(); // Clear buffer to prevent re-trying same bad state
    return NULL; // Early return avoids 'goto' crossing initialization issues
  }

  DEBUG_FRAME(std::cout << "[DEBUG] Copying " << total_size
              << " bytes from vecChar[0]" << std::endl);

  memcpy(pf, &vecChar[0], total_size);

  current_frame_end_offset = FRAME_HEAD_SIZE + frame_payload_len;
  remaining_start_index = current_frame_end_offset + FRAME_CHECKSUM_SIZE + FRAME_END_SIZE;

  if (remaining_start_index > vecChar.size()) {
    std::cerr << "[ERROR] Calculated remaining start index ("
              << remaining_start_index << ") exceeds vector size ("
              << vecChar.size() << "). Dropping frame." << std::endl;
    free(pf); // Clean up allocated memory on error path
    return NULL; // Early return avoids 'goto' crossing initialization issues
  }

  DEBUG_FRAME(std::cout << "[DEBUG] Remaining data starts at index "
              << remaining_start_index
              << ". Keeping " << (vecChar.size() - remaining_start_index)
              << " bytes." << std::endl);

  // Create new vector with only the tail data to keep for next parse cycle
  std::vector<uint8_t>(vecChar.begin() + remaining_start_index, vecChar.end())
      .swap(vecChar);

  return pf;
  // --- Safety Block End ---

__finished:
  return NULL;
}
