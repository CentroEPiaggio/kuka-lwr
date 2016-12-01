///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF, Inc. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Wim Meeussen

/// \author Hamal Marino
/// 2016-10: modified the file "joint_state_interface.h" to be an interface for cartesian variables

#ifndef HARDWARE_INTERFACE_CARTESIAN_COMMAND_INTERFACE_H
#define HARDWARE_INTERFACE_CARTESIAN_COMMAND_INTERFACE_H

#include <cassert>
#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <cartesian_hardware_interface/cartesian_state_interface.h>
#include <hardware_interface/joint_command_interface.h>

namespace hardware_interface
{

/** \brief A handle used to read and command a single cartesian variable. */
class CartesianVariableHandle : public CartesianStateHandle
{
public:
  CartesianVariableHandle() : CartesianStateHandle(), cmd_(0) {}

  /**
   * \param js This cartesian variable's state handle
   * \param cmd A pointer to the storage for this cartesian variable's output command
   */
  CartesianVariableHandle(const CartesianStateHandle& js, double* cmd)
    : CartesianStateHandle(js), cmd_(cmd)
  {
    if (!cmd_)
    {
      throw HardwareInterfaceException("Cannot create handle '" + js.getName() + "'. Command data pointer is null.");
    }
  }

  void setCommand(double command) {assert(cmd_); *cmd_ = command;}
  double getCommand() const {assert(cmd_); return *cmd_;}

private:
  double* cmd_;
};

/** \brief Hardware interface to support commanding an array of cartesian variables.
 *
 * This \ref HardwareInterface supports commanding the output of an array of
 * named cartesian variables. Note that these commands can have any semantic meaning as long
 * as they each can be represented by a single double, they are not necessarily
 * effort commands. To specify a meaning to this command, see the derived
 * classes like \ref EffortJointInterface etc.
 * 
 * This class inherits from two different HW resource manager in order to be able to have
 * information also about joints, and not just about cartesian variables.
 * Moreover, even if this class controls cartesian variables, it has to clam also associated joints
 * as no other controller should be able to use them.
 *
 * \note Getting a cartesian variable handle through the getHandle() method \e will claim that resource.
 *
 */
class CartesianCommandInterface : public HardwareResourceManager<CartesianVariableHandle, ClaimResources>, 
                                  public HardwareResourceManager<JointHandle, ClaimResources>
{
public:
    /// these "using" directives are needed to disambiguate
    using HardwareResourceManager<CartesianVariableHandle, ClaimResources>::ResourceManager<CartesianVariableHandle>::registerHandle;
    using HardwareResourceManager<JointHandle, ClaimResources>::ResourceManager<JointHandle>::registerHandle;
    
    /// getHandle needs to be discriminated as there is no way of deducing which functions to call (only differ based on return type)
    /// unless using a Proxy class, and exploiting the cast operator
    class handleProxy
    {
        CartesianCommandInterface* myOwner;
        const std::string& myName;
    public:
        handleProxy( CartesianCommandInterface* owner, const std::string& name ) : myOwner(owner), myName(name) {}
        /// the commented implementation is more generic, and more error prone: could try to call also different cast,
        /// and may thus result in inconsistencies (as HardwareResourceManager<T,ClaimResources> only works for some T's
        // template<class T>
        // operator T() const
        // {
        //     return myOwner->HardwareResourceManager<T, ClaimResources>::getHandle(myName);
        // }
        operator CartesianVariableHandle() const
        {
            return myOwner->HardwareResourceManager<CartesianVariableHandle, ClaimResources>::getHandle(myName);
        }
        operator JointHandle() const
        {
            return myOwner->HardwareResourceManager<JointHandle, ClaimResources>::getHandle(myName);
        }
    };
    
    handleProxy getHandle(const std::string& name)
    {
        return handleProxy(this, name);
    }
    
    /// get names for all resources
    std::vector<std::string> getNames() const
    {
        std::vector<std::string> out1 = this->HardwareResourceManager<JointHandle, ClaimResources>::getNames();
        std::vector<std::string> out2 = this->HardwareResourceManager<CartesianVariableHandle, ClaimResources>::getNames();
        out1.insert(out1.end(), std::make_move_iterator(out2.begin()), std::make_move_iterator(out2.end()));
        return out1;
    }
    
    /// Clear the resources this interface is claiming
    void clearClaims()
    {
        this->HardwareResourceManager<JointHandle, ClaimResources>::clearClaims();
        this->HardwareResourceManager<CartesianVariableHandle, ClaimResources>::clearClaims();
        return;
    }
    
    /// Get the list of resources this interface is currently claiming
    std::set<std::string> getClaims() const
    {
        std::set<std::string> out1 = this->HardwareResourceManager<JointHandle, ClaimResources>::getClaims();
        std::set<std::string> out2 = this->HardwareResourceManager<CartesianVariableHandle, ClaimResources>::getClaims();
        out1.insert(out2.begin(), out2.end());
        return out1;
    }
};

/// \ref CartesianCommandInterface for commanding cartesian-based joints.
class PositionCartesianInterface : public CartesianCommandInterface {};
}

#endif
