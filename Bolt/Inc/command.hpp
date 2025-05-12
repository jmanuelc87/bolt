#ifndef BOLT_COMMAND_H
#define BOLT_COMMAND_H

namespace bolt
{
    class Command
    {
    public:
        virtual void execute() = 0;
        virtual ~Command() = default;
    };
};

#endif /* BOLT_COMMAND_H */
