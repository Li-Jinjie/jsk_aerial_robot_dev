"""
 Created by li-jinjie on 2025/12/7.
"""
allowed_token_list = ["g4", "a4", "b4", "c5", "d5"]  # allowed note strings


def tokenize(user_input, allowed_tokens):
    tokens = []
    i = 0
    L = len(user_input)

    allowed_tokens = sorted(allowed_tokens, key=len, reverse=True)

    while i < L:
        matched = False
        for token in allowed_tokens:
            if user_input.startswith(token, i):
                tokens.append(token)
                i += len(token)
                matched = True
                break
        if not matched:
            raise ValueError(f"Invalid token starting at position {i}: '{user_input[i:]}'")

    return tokens
