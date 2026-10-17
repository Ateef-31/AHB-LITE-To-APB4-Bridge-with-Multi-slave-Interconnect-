set search_path  [list . ../src ./script /fetools/work_area/frontend/SAED32_EDK/lib/stdcell_lvt/db_nldm /fetools/work_area/frontend/SAED32_EDK/lib/stdcell_hvt/db_nldm]
set target_library [list saed32hvt_ss0p75v25c.db saed32lvt_ss0p75v25c.db]
set link_library "* $target_library "



