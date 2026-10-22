#include "identify.h"

IdentResult_TypeDef gIdent;

IdentState_TypeDef Ident_Request(void)
{
    /* TODO(P3)：见 identify.h 的方法说明。需要 supervisor 增加一个
     * MainState_Ident 或 Run 子状态 RunState_Ident，并由它接管 PWM。 */
    gIdent.state = IDENT_NOT_IMPLEMENTED;
    return gIdent.state;
}
