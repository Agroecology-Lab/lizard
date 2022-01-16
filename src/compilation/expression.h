#pragma once

#include "type.h"
#include "variable.h"
#include <memory>
#include <string>

class Expression;
using Expression_ptr = std::shared_ptr<Expression>;
using ConstExpression_ptr = std::shared_ptr<const Expression>;

class Expression {
protected:
    Expression(const Type type);

public:
    const Type type;

    virtual bool evaluate_boolean() const;
    virtual int64_t evaluate_integer() const;
    virtual double evaluate_number() const;
    virtual std::string evaluate_string() const;
    virtual std::string evaluate_identifier() const;
    bool is_numbery() const;
    int print_to_buffer(char *buffer) const;
};

class BooleanExpression : public Expression {
private:
    const bool value;

public:
    BooleanExpression(const bool value);
    bool evaluate_boolean() const;
};

class StringExpression : public Expression {
private:
    const std::string value;

public:
    StringExpression(const std::string value);
    std::string evaluate_string() const;
};

class IntegerExpression : public Expression {
private:
    const int64_t value;

public:
    IntegerExpression(const int64_t value);
    int64_t evaluate_integer() const;
    double evaluate_number() const;
};

class NumberExpression : public Expression {
private:
    const double value;

public:
    NumberExpression(const double value);
    double evaluate_number() const;
};

class VariableExpression : public Expression {
private:
    const ConstVariable_ptr variable;

public:
    VariableExpression(const ConstVariable_ptr variable);
    bool evaluate_boolean() const;
    int64_t evaluate_integer() const;
    double evaluate_number() const;
    std::string evaluate_string() const;
    std::string evaluate_identifier() const;
};

class PowerExpression : public Expression {
private:
    const ConstExpression_ptr left;
    const ConstExpression_ptr right;

public:
    PowerExpression(const ConstExpression_ptr left, const ConstExpression_ptr right);
    int64_t evaluate_integer() const;
    double evaluate_number() const;
};

class NegateExpression : public Expression {
private:
    const ConstExpression_ptr operand;

public:
    NegateExpression(const ConstExpression_ptr operand);
    int64_t evaluate_integer() const;
    double evaluate_number() const;
};

class MultiplyExpression : public Expression {
private:
    const ConstExpression_ptr left;
    const ConstExpression_ptr right;

public:
    MultiplyExpression(const ConstExpression_ptr left, const ConstExpression_ptr right);
    int64_t evaluate_integer() const;
    double evaluate_number() const;
};

class DivideExpression : public Expression {
private:
    const ConstExpression_ptr left;
    const ConstExpression_ptr right;

public:
    DivideExpression(const ConstExpression_ptr left, const ConstExpression_ptr right);
    int64_t evaluate_integer() const;
    double evaluate_number() const;
};

class AddExpression : public Expression {
private:
    const ConstExpression_ptr left;
    const ConstExpression_ptr right;

public:
    AddExpression(const ConstExpression_ptr left, const ConstExpression_ptr right);
    int64_t evaluate_integer() const;
    double evaluate_number() const;
};

class SubtractExpression : public Expression {
private:
    const ConstExpression_ptr left;
    const ConstExpression_ptr right;

public:
    SubtractExpression(const ConstExpression_ptr left, const ConstExpression_ptr right);
    int64_t evaluate_integer() const;
    double evaluate_number() const;
};

class ShiftLeftExpression : public Expression {
private:
    const ConstExpression_ptr left;
    const ConstExpression_ptr right;

public:
    ShiftLeftExpression(const ConstExpression_ptr left, const ConstExpression_ptr right);
    int64_t evaluate_integer() const;
};

class ShiftRightExpression : public Expression {
private:
    const ConstExpression_ptr left;
    const ConstExpression_ptr right;

public:
    ShiftRightExpression(const ConstExpression_ptr left, const ConstExpression_ptr right);
    int64_t evaluate_integer() const;
};

class BitAndExpression : public Expression {
private:
    const ConstExpression_ptr left;
    const ConstExpression_ptr right;

public:
    BitAndExpression(const ConstExpression_ptr left, const ConstExpression_ptr right);
    int64_t evaluate_integer() const;
};

class BitXorExpression : public Expression {
private:
    const ConstExpression_ptr left;
    const ConstExpression_ptr right;

public:
    BitXorExpression(const ConstExpression_ptr left, const ConstExpression_ptr right);
    int64_t evaluate_integer() const;
};

class BitOrExpression : public Expression {
private:
    const ConstExpression_ptr left;
    const ConstExpression_ptr right;

public:
    BitOrExpression(const ConstExpression_ptr left, const ConstExpression_ptr right);
    int64_t evaluate_integer() const;
};

class GreaterExpression : public Expression {
private:
    const ConstExpression_ptr left;
    const ConstExpression_ptr right;

public:
    GreaterExpression(const ConstExpression_ptr left, const ConstExpression_ptr right);
    bool evaluate_boolean() const;
};

class LessExpression : public Expression {
private:
    const ConstExpression_ptr left;
    const ConstExpression_ptr right;

public:
    LessExpression(const ConstExpression_ptr left, const ConstExpression_ptr right);
    bool evaluate_boolean() const;
};

class GreaterEqualExpression : public Expression {
private:
    const ConstExpression_ptr left;
    const ConstExpression_ptr right;

public:
    GreaterEqualExpression(const ConstExpression_ptr left, const ConstExpression_ptr right);
    bool evaluate_boolean() const;
};

class LessEqualExpression : public Expression {
private:
    const ConstExpression_ptr left;
    const ConstExpression_ptr right;

public:
    LessEqualExpression(const ConstExpression_ptr left, const ConstExpression_ptr right);
    bool evaluate_boolean() const;
};

class EqualExpression : public Expression {
private:
    const ConstExpression_ptr left;
    const ConstExpression_ptr right;

public:
    EqualExpression(const ConstExpression_ptr left, const ConstExpression_ptr right);
    bool evaluate_boolean() const;
};

class UnequalExpression : public Expression {
private:
    const ConstExpression_ptr left;
    const ConstExpression_ptr right;

public:
    UnequalExpression(const ConstExpression_ptr left, const ConstExpression_ptr right);
    bool evaluate_boolean() const;
};

class NotExpression : public Expression {
private:
    const ConstExpression_ptr operand;

public:
    NotExpression(const ConstExpression_ptr operand);
    bool evaluate_boolean() const;
};

class AndExpression : public Expression {
private:
    const ConstExpression_ptr left;
    const ConstExpression_ptr right;

public:
    AndExpression(const ConstExpression_ptr left, const ConstExpression_ptr right);
    bool evaluate_boolean() const;
};

class OrExpression : public Expression {
private:
    const ConstExpression_ptr left;
    const ConstExpression_ptr right;

public:
    OrExpression(const ConstExpression_ptr left, const ConstExpression_ptr right);
    bool evaluate_boolean() const;
};
