#ifndef ASTRA_VERSION_H_
#define ASTRA_VERSION_H_

#define ASTRA_ROS_MAJOR_VERSION 0
#define ASTRA_ROS_MINOR_VERSION 3
#define ASTRA_ROS_PATCH_VERSION 1

#ifndef STRINGIFY
#define STRINGIFY(arg) #arg
#endif
#ifndef VAR_ARG_STRING
#define VAR_ARG_STRING(arg) STRINGIFY(arg)
#endif

/* Versioning rules            : For each release at least one of [MJR/MNR/PTCH]
 * triple is promoted */
/*                             : Versions that differ by OB_API_PATCH_VERSION
 * only are interface-compatible, i.e. no user-code changes required */
/*                             : Versions that differ by MAJOR/MINOR VERSION
 * component can introduce API changes */
/* Version in encoded integer format (1,0,x) -> 01000x. note that each component
 * is limited into [0-99] range by design
 */
#define ASTRA_ROS_VERSION                                                \
  (((ASTRA_API_MAJOR_VERSION)*10000) + ((ASTRA_API_MINOR_VERSION)*100) + \
   (ASTRA_API_PATCH_VERSION))
/* Return version in "X.Y.Z" format */
#define ASTRA_ROS_VERSION_STR                                     \
  (VAR_ARG_STRING(ASTRA_ROS_MAJOR_VERSION.ASTRA_ROS_MINOR_VERSION \
                      .ASTRA_ROS_PATCH_VERSION))

#endif  // ASTRA_VERSION_H_